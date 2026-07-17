// ikd_Tree.h - ikd-Tree: 增量KD树数据结构
// 作者: Yixi Cai (yixicai@connect.hku.hk)
// 论文: ikd-Tree: An Incremental KD Tree for Robotic Applications
// 功能：
//   1. 高效的增量式KD树：支持在线点云插入、删除、搜索
//   2. 惰性删除策略：被删除点标记为invalid，累积到一定比例后再重建
//   3. 多线程重建：大规模重建在后台线程进行，不影响前台搜索
//   4. 体素降采样：添加点时自动进行体素降采样，保持地图密度均匀
//   5. 支持盒搜索（Box Search）、半径搜索（Radius Search）、最近邻搜索（KNN）

#pragma once
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <algorithm>
#include <memory.h>
#include <pcl/point_types.h>

// --- 常量定义 ---
#define EPSS 1e-6                           // 浮点数比较精度
#define Minimal_Unbalanced_Tree_Size 10      // 最小不平衡树大小（小于此值不触发重建）
#define Multi_Thread_Rebuild_Point_Num 1500  // 多线程重建的最小点数阈值
#define DOWNSAMPLE_SWITCH true              // 降采样开关
#define ForceRebuildPercentage 0.2          // 强制重建比例
#define Q_LEN 1000000                       // 操作日志队列最大长度

using namespace std;

// --- 轴对齐包围盒结构体 ---
struct BoxPointType
{
    float vertex_min[3];  // 包围盒最小角点 (x_min, y_min, z_min)
    float vertex_max[3];  // 包围盒最大角点 (x_max, y_max, z_max)
};

// --- 操作类型枚举（操作日志记录） ---
enum operation_set
{
    ADD_POINT,          // 添加点
    DELETE_POINT,       // 删除点
    DELETE_BOX,         // 删除盒内所有点
    ADD_BOX,            // 恢复盒内被删除的点
    DOWNSAMPLE_DELETE,  // 降采样删除（替换）
    PUSH_DOWN           // 向下传播删除标记
};

// --- 删除点存储类型（flatten展平时的记录策略） ---
enum delete_point_storage_set
{
    NOT_RECORD,          // 不记录被删除的点
    DELETE_POINTS_REC,   // 记录被删除的点（单线程重建使用）
    MULTI_THREAD_REC     // 记录被删除的点（多线程重建使用）
};

template <typename PointType>
class KD_TREE
{
public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<KD_TREE<PointType>>;

    // === KD树节点结构 ===
    struct KD_TREE_NODE
    {
        PointType point;                      // 该节点存储的点
        int division_axis;                    // 分割轴（0:x, 1:y, 2:z）
        int TreeSize = 1;                     // 子树中总节点数（含自己）
        int invalid_point_num = 0;            // 子树中标记为删除的点数
        int down_del_num = 0;                 // 子树中因降采样被删除的点数

        // --- 惰性删除标记 ---
        bool point_deleted = false;           // 当前点是否已标记删除
        bool tree_deleted = false;            // 整个子树是否已标记删除（含点和降采样删除）
        bool point_downsample_deleted = false; // 当前点是否因降采样被标记删除
        bool tree_downsample_deleted = false;  // 整个子树是否因降采样被标记删除

        // --- 向下传播标记（惰性删除信息向下传播到子节点） ---
        bool need_push_down_to_left = false;   // 是否需要向左子节点传播删除信息
        bool need_push_down_to_right = false;  // 是否需要向右子节点传播删除信息
        bool working_flag = false;             // 节点是否正在被操作（线程安全标志）

        pthread_mutex_t push_down_mutex_lock;  // 传播操作的互斥锁

        // --- 子树空间范围 ---
        float node_range_x[2], node_range_y[2], node_range_z[2];  // 子树包围盒范围 [min, max]
        float radius_sq;                       // 包围盒外接球半径的平方

        KD_TREE_NODE *left_son_ptr = nullptr;   // 左子节点
        KD_TREE_NODE *right_son_ptr = nullptr;  // 右子节点
        KD_TREE_NODE *father_ptr = nullptr;     // 父节点

        // --- 统计数据（论文记录用） ---
        float alpha_del;   // 删除比例 = invalid_point_num / TreeSize
        float alpha_bal;   // 不平衡度 = max(left_size, right_size) / (TreeSize - 1)
    };

    // --- 操作日志结构体：记录一次操作，用于后台重建时重放 ---
    struct Operation_Logger_Type
    {
        PointType point;                      // 操作涉及的单个点
        BoxPointType boxpoint;                // 操作涉及的包围盒
        bool tree_deleted;                    // 树删除标记
        bool tree_downsample_deleted;         // 树降采样删除标记
        operation_set op;                     // 操作类型
    };

    // --- 点类型比较器：按距离排序，用于最近邻搜索堆 ---
    struct PointType_CMP
    {
        PointType point;
        float dist = 0.0;
        PointType_CMP(PointType p = PointType(), float d = INFINITY)
        {
            this->point = p;
            this->dist = d;
        };
        bool operator<(const PointType_CMP &a) const
        {
            if (fabs(dist - a.dist) < 1e-10)   // 距离相同按x坐标决定
                return point.x < a.point.x;
            else
                return dist < a.dist;           // 按距离排序
        }
    };

    // === 手动实现的最大堆 ===
    // 用于最近邻搜索时的k近邻维护（大顶堆，堆顶是当前最远的点）
    class MANUAL_HEAP
    {
    public:
        MANUAL_HEAP(int max_capacity = 100)
        {
            cap = max_capacity;
            heap = new PointType_CMP[max_capacity];
            heap_size = 0;
        }

        ~MANUAL_HEAP() { delete[] heap; }

        void pop()    // 弹出堆顶（最大元素）
        {
            if (heap_size == 0) return;
            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);
            return;
        }

        PointType_CMP top() { return heap[0]; }  // 查看堆顶

        void push(PointType_CMP point)           // 入堆
        {
            if (heap_size >= cap) return;
            heap[heap_size] = point;
            FloatUp(heap_size);
            heap_size++;
            return;
        }

        int size() { return heap_size; }
        void clear() { heap_size = 0; return; }

    private:
        PointType_CMP *heap;
        void MoveDown(int heap_index)
        {
            int l = heap_index * 2 + 1;
            PointType_CMP tmp = heap[heap_index];
            while (l < heap_size)
            {
                if (l + 1 < heap_size && heap[l] < heap[l + 1])
                    l++;
                if (tmp < heap[l])
                {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
        }
        void FloatUp(int heap_index)
        {
            int ancestor = (heap_index - 1) / 2;
            PointType_CMP tmp = heap[heap_index];
            while (heap_index > 0)
            {
                if (heap[ancestor] < tmp)
                {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
        }
        int heap_size = 0;
        int cap = 0;
    };

    // === 手动实现的循环队列（操作日志队列） ===
    class MANUAL_Q
    {
    private:
        int head = 0, tail = 0, counter = 0;
        Operation_Logger_Type q[Q_LEN];
        bool is_empty = true;

    public:
        void pop()
        {
            if (counter == 0) return;
            head++;
            head %= Q_LEN;
            counter--;
            if (counter == 0) is_empty = true;
            return;
        }
        Operation_Logger_Type front() { return q[head]; }
        Operation_Logger_Type back()  { return q[tail]; }
        void clear() { head = 0; tail = 0; counter = 0; is_empty = true; return; }
        void push(Operation_Logger_Type op)
        {
            q[tail] = op;
            counter++;
            if (is_empty) is_empty = false;
            tail++;
            tail %= Q_LEN;
        }
        bool empty() { return is_empty; }
        int size()  { return counter; }
    };

private:
    // === 多线程重建相关变量 ===
    bool termination_flag = false;             // 终止重建线程标志
    bool rebuild_flag = false;                 // 是否正在进行重建
    pthread_t rebuild_thread;                  // 重建线程
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    MANUAL_Q Rebuild_Logger;                   // 重建期间缓存的操作日志
    PointVector Rebuild_PCL_Storage;           // 重建用的点云存储
    KD_TREE_NODE **Rebuild_Ptr = nullptr;      // 需要重建的子树根节点指针的指针
    int search_mutex_counter = 0;              // 搜索计数器（用于读写锁模拟）

    // 多线程重建相关函数
    static void *multi_thread_ptr(void *arg);  // 重建线程入口
    void multi_thread_rebuild();               // 多线程重建主函数
    void start_thread();                       // 启动重建线程
    void stop_thread();                        // 停止重建线程
    void run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation);  // 重放一条操作日志

    // === KD树核心参数 ===
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;       // 删除比例阈值（超过则触发重建）
    float balance_criterion_param = 0.7f;      // 平衡度阈值（超过则触发重建）
    float downsample_size = 0.2f;              // 降采样体素大小
    bool Delete_Storage_Disabled = false;      // 是否禁用删除点存储
    KD_TREE_NODE *STATIC_ROOT_NODE = nullptr;  // 静态根节点（根节点的父亲）

    // === 删除点存储容器 ===
    PointVector Points_deleted;                // 被删除的点（单线程重构用）
    PointVector Downsample_Storage;            // 降采样临时存储
    PointVector Multithread_Points_deleted;    // 被删除的点（多线程重构用）

    // === KD树内部函数 ===
    void InitTreeNode(KD_TREE_NODE *root);                                     // 初始化节点
    void Test_Lock_States(KD_TREE_NODE *root);                                 // 测试锁状态
    void BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage);  // 递归构建KD树
    void Rebuild(KD_TREE_NODE **root);                                         // 重建子树（单/多线程）
    int Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample); // 按范围删除
    void Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild); // 按点删除
    void Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis);          // 按点添加
    void Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild);                    // 按范围恢复
    void Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, float max_dist);      // KNN搜索
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);                 // 盒搜索
    void Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage);        // 半径搜索
    bool Criterion_Check(KD_TREE_NODE *root);   // 重建准则检查（平衡度和删除率）
    void Push_Down(KD_TREE_NODE *root);          // 向下传播删除/降采样标记
    void Update(KD_TREE_NODE *root);             // 更新节点的size、范围、alpha等统计信息
    void delete_tree_nodes(KD_TREE_NODE **root);  // 递归删除树节点
    void downsample(KD_TREE_NODE **root);         // 降采样处理
    bool same_point(PointType a, PointType b);    // 检查两点是否相同
    float calc_dist(PointType a, PointType b);    // 计算两点距离平方
    float calc_box_dist(KD_TREE_NODE *node, PointType point); // 计算点到包围盒距离平方
    static bool point_cmp_x(PointType a, PointType b); // 按x坐标比较
    static bool point_cmp_y(PointType a, PointType b); // 按y坐标比较
    static bool point_cmp_z(PointType a, PointType b); // 按z坐标比较

public:
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6, float box_length = 0.2);
    ~KD_TREE();

    // --- 参数设置 ---
    void Set_delete_criterion_param(float delete_param) { delete_criterion_param = delete_param; }
    void Set_balance_criterion_param(float balance_param) { balance_criterion_param = balance_param; }
    void set_downsample_param(float downsample_param) { downsample_size = downsample_param; }
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);

    // --- 查询函数 ---
    int size();                   // 返回树的总节点数
    int validnum();              // 返回树的有效节点数
    void root_alpha(float &alpha_bal, float &alpha_del); // 获取根节点的平衡度和删除率

    // --- 构建函数 ---
    void Build(PointVector point_cloud);  // 从点云构建KD树

    // --- 搜索函数 ---
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, float max_dist = INFINITY); // K最近邻搜索
    void Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage);  // 盒范围搜索
    void Radius_Search(PointType point, const float radius, PointVector &Storage); // 半径搜索

    // --- 修改函数 ---
    int Add_Points(PointVector &PointToAdd, bool downsample_on);                          // 批量添加点（返回降采样操作数）
    void Add_Point_Boxes(vector<BoxPointType> &BoxPoints);                                // 批量恢复盒内点
    void Delete_Points(PointVector &PointToDel);                                          // 批量删除点
    int Delete_Point_Boxes(vector<BoxPointType> &BoxPoints);                              // 批量删除盒内点（返回删除数）

    // --- 展平和辅助函数 ---
    void flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type); // 展平子树到容器
    void acquire_removed_points(PointVector &removed_points);                          // 获取最近被删除的点
    BoxPointType tree_range();                                                          // 获取树的包围盒范围

    // --- 公开成员 ---
    PointVector PCL_Storage;          // 展平时的临时存储
    KD_TREE_NODE *Root_Node = nullptr; // 树根节点
    int max_queue_size = 0;           // 重建日志队列最大长度（统计用）
};
