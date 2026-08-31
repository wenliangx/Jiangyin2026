# =============================================================================
# RA-LIO 系统架构与数据流图
# =============================================================================
#
# ┌─────────────────────────────────────────────────────────────────────────┐
# │                          RA-LIO 系统概览                                │
# │         基于 FAST-LIO2 的 LiDAR-IMU 紧耦合迭代卡尔曼滤波里程计            │
# └─────────────────────────────────────────────────────────────────────────┘
#
# 一、文件依赖关系图
# ==================
#
# 现代化后的运行边界：
#
#   laser_mapping.cpp (仅 main)
#          │
#          ▼
#   MappingNode ── MappingConfig
#      │   │   │
#      │   │   ├── Preprocessor（五类雷达统一入口）
#      │   ├────── ImuProcessor（PImpl，初始化/预测/去畸变）
#      └────────── ErrorStateKalmanFilter + IncrementalMap
#                                      │
#                                      └── ikd-Tree（隔离的上游实现）
#
# 第一方代码统一使用 C++17 和 `ra_lio` 命名空间；ROS 节点名、参数、话题、
# frame 与 TF 契约保持不变。
#
#                              ┌──────────────┐
#                             │common_lib.hpp│  ◄── 公共类型/宏/工具函数
#                              └──────┬───────┘
#                ┌────────────────────┼────────────────────┐
#                ▼                    ▼                    ▼
#        ┌──────────────┐   ┌──────────────┐     ┌──────────────┐
#        │use_ikfom.hpp │   │imu_processing│     │preprocess.hpp│
#        │ 状态定义+模型 │   │    .hpp      │     │  点云预处理   │
#        └──────┬───────┘   └──────┬───────┘     └──────┬───────┘
#               │                  │                    │
#               ▼                  │                    ▼
#        ┌──────────────┐         │            ┌──────────────┐
#        │ esekfom.hpp  │◄────────┘            │preprocess.cpp│
#        │  ESKF滤波器   │                      │  预处理实现   │
#        └──────┬───────┘                      └──────┬───────┘
#               │                                     │
#               │         ┌──────────────┐            │
#               └────────►│laserMapping  │◄───────────┘
#                         │    .cpp      │
#                         │  主建图节点   │
#                         └──────┬───────┘
#                                │
#                    ┌───────────┼───────────┐
#                    ▼           ▼           ▼
#             ┌──────────┐┌──────────┐┌──────────┐
#             │ikd_Tree.h││ikd_Tree  ││   ROS    │
#             │  KD树声明 ││  .cpp    ││ 消息发布  │
#             └──────────┘└──────────┘└──────────┘
#
#
# 二、数据流图（主循环处理流程）
# ==============================
#
#    ┌─────────┐     ┌─────────┐
#    │  LiDAR  │     │   IMU   │
#    │  传感器  │     │  传感器  │
#    └────┬────┘     └────┬────┘
#         │               │
#    ┌────▼────────┐ ┌───▼──────────┐
#    │标准/custom  │ │  imu_cbk()   │  ROS回调线程
#    │ pcl_cbk()   │ │              │
#    └────┬────────┘ └───┬──────────┘
#         │               │
#    ┌────▼────────┐      │
#    │preprocess:: │      │
#    │  process()  │      │
#    │ 格式转换/   │      │
#    │ 降采样/盲区 │      │
#    └────┬────────┘      │
#         │               │
#    ┌────▼────────┐ ┌───▼──────────┐
#    │lidar_buffer │ │ imu_buffer   │  带互斥锁的线程安全队列
#    │ +time_buffer│ │              │
#    └────┬────────┘ └───┬──────────┘
#         │               │
#         └───────┬───────┘
#                 │
#      ┌──────────▼───────────┐
#      │  sync_packages()     │  数据同步：取一帧LiDAR + 对应时间的IMU
#      │  → MeasureGroup      │
#      └──────────┬───────────┘
#                 │
#      ┌──────────▼───────────┐
#      │  ImuProcessor::      │
#      │    Process()         │
#      │                      │
#      │  ┌─ IMU_init()       │  初始化阶段：重力对齐、bias估计
#      │  │  (前20帧)          │
#      │  └─────────────────── │
#      │  ┌─ 坐标变换          │  正常阶段：P_lidar → P_imu → P_world
#      │  │  P_world =         │
#      │  │  R*(R_ext*P+t)+pos │
#      │  └─────────────────── │
#      └──────────┬───────────┘
#                 │
#       feats_undistort (世界系点云)
#                 │
#      ┌──────────▼───────────┐
#      │  VoxelGrid 降采样    │  downSizeFilterSurf
#      │  feats_undistort     │
#      │      ↓               │
#      │  feats_down_body     │  LiDAR/body系 降采样后点云
#      └──────────┬───────────┘
#                 │
#      ┌──────────▼───────────┐
#      │  lasermap_fov_       │  局部地图FOV管理
#      │    segment()         │  检查是否需要移动局部地图
#      │                      │  超出范围的点 → cub_needrm
#      └──────────┬───────────┘
#                 │
#      ┌──────────▼─────────────────────────────────┐
#      │  ESKF 迭代更新                              │
#      │  kf.update_iterated_dyn_share_modified()   │
#      │                                             │
#      │  for i in 0..max_iter:                     │
#      │    ┌─ h_share_model() ──────────────────┐  │
#      │    │  对每个特征点:                       │  │
#      │    │   1. P_body → P_world              │  │
#      │    │   2. ikdtree.Nearest_Search() 找5近邻│  │
#      │    │   3. esti_plane() 拟合平面          │  │
#      │    │   4. 计算残差 h = -点到平面距离      │  │
#      │    │   5. 计算雅可比 H = dh/dx           │  │
#      │    └────────────────────────────────────┘  │
#      │    ┌─ 卡尔曼更新 ───────────────────────┐  │
#      │    │  K = (HᵀH/R + P⁻¹)⁻¹Hᵀ/R         │  │
#      │    │  x = x ⊞ (K·h + (KH-I)·dx)        │  │
#      │    │  检查收敛 |dx| < epsi              │  │
#      │    └────────────────────────────────────┘  │
#      │  P = (I-KH)P                              │
#      └──────────┬─────────────────────────────────┘
#                 │
#      更新后状态: state_point (pos, rot, vel, bg, ba, grav)
#                 │
#     ┌───────────┼───────────────────┐
#     │           │                   │
# ┌───▼────┐ ┌───▼────────┐  ┌───────▼──────┐
# │里程计  │ │ map_       │  │ 点云/路径/   │
# │发布    │ │ incremental│  │ 速度可视化   │
# │        │ │            │  │ 发布         │
# │odom    │ │add points  │  │              │
# │ + TF   │ │to ikd-tree │  │cloud/path/   │
# │        │ │            │  │marker        │
# └────────┘ └────────────┘  └──────────────┘
#
#
# 三、状态量结构图（24维）
# =======================
#
#    state_ikfom x_ (24维向量)
#    ┌──────────────────────────────────────────────────────┐
#    │ 索引     │ 含义              │ 类型      │ 维度     │
#    ├──────────┼───────────────────┼───────────┼──────────┤
#    │  0:3     │ pos   位置(W系)   │ Vector3d  │    3     │
#    │  3:6     │ rot   姿态(SO3)   │ SO3d      │    3*    │
#    │  6:9     │ R_ext 外参旋转    │ SO3d      │    3*    │
#    │  9:12    │ T_ext 外参平移    │ Vector3d  │    3     │
#    │ 12:15    │ vel   速度(W系)   │ Vector3d  │    3     │
#    │ 15:18    │ bg    陀螺仪bias  │ Vector3d  │    3     │
#    │ 18:21    │ ba    加速度bias  │ Vector3d  │    3     │
#    │ 21:24    │ grav  重力向量    │ Vector3d  │    3     │
#    └──────────┴───────────────────┴───────────┴──────────┘
#    * SO(3)在切空间中以3维李代数表示，名义状态为3x3旋转矩阵
#
#    误差状态 δx ∈ R²⁴（用于卡尔曼滤波更新）
#    ┌──────────┬─────────────────────┬──────────────┐
#    │  0:3     │ δp    位置误差      │ 普通向量加法  │
#    │  3:6     │ δθ    姿态误差      │ SO(3)指数映射 │
#    │  6:9     │ δθ_ext 外参旋转误差 │ SO(3)指数映射 │
#    │  9:12    │ δt_ext 外参平移误差 │ 普通向量加法  │
#    │ 12:15    │ δv    速度误差      │ 普通向量加法  │
#    │ 15:18    │ δbg   陀螺仪bias误差│ 普通向量加法  │
#    │ 18:21    │ δba   加速度bias误差│ 普通向量加法  │
#    │ 21:24    │ δg    重力向量误差  │ 普通向量加法  │
#    └──────────┴─────────────────────┴──────────────┘
#
#
# 四、关键数据流 Mermaid 图（可在支持Mermaid的阅读器中渲染）
# ========================================================
#
# ```mermaid
# flowchart TD
#     subgraph 传感器输入
#         L[LiDAR 激光雷达] -->|CustomMsg/PointCloud2| CB_L[lidar_cbk]
#         I[IMU 惯性测量单元] -->|sensor_msgs::Imu| CB_I[imu_cbk]
#     end
#
#     subgraph 数据预处理 preprocess.cpp
#         CB_L --> PP[Preprocessor::process<br/>格式转换/降采样/盲区过滤]
#         PP --> LB[lidar_buffer + time_buffer]
#         CB_I --> IB[imu_buffer]
#     end
#
#     subgraph 数据同步 mapping_node.cpp
#         LB --> SP[sync_packages<br/>一帧LiDAR+对应IMU]
#         IB --> SP
#         SP --> MG[MeasureGroup]
#     end
#
#     subgraph IMU处理 imu_processing.hpp
#         MG --> IP{imu_need_init?}
#         IP -->|是| II[IMU_init<br/>重力对齐/bias估计]
#         IP -->|否| CT[坐标变换<br/>P_lidar→P_imu→P_world]
#         II -->|初始化完成| IP
#         CT --> FU[feats_undistort<br/>世界系去畸变点云]
#     end
#
#     subgraph ESKF滤波器 esekfom.hpp
#         FU --> DS[VoxelGrid降采样<br/>→ feats_down_body]
#         DS --> FOV[lasermap_fov_segment<br/>局部地图范围管理]
#         FOV --> PRED[kf.predict<br/>IMU前向传播]
#         PRED --> UPD[kf.update_iterated<br/>点到平面残差IEKF更新]
#
#         subgraph 观测模型
#             UPD --> HS[h_share_model<br/>对每个特征点：]
#             HS --> KNN[ikdtree.Nearest_Search<br/>找5个最近邻]
#             KNN --> EP[esti_plane<br/>拟合平面求法向量]
#             EP --> JAC[计算雅可比H + 残差h]
#         end
#
#         JAC --> KF[卡尔曼增益+状态更新<br/>收敛检查]
#         KF -->|未收敛| HS
#         KF -->|收敛| SP2[state_point<br/>更新后状态]
#     end
#
#     subgraph 地图管理 ikd-Tree
#         SP2 --> MI[map_incremental<br/>点加入全局地图]
#         MI --> IKDT[ikdtree.Add_Points<br/>体素降采样+插入]
#         FOV --> DEL[ikdtree.Delete_Point_Boxes<br/>删除超出FOV的点]
#     end
#
#     subgraph 发布输出 mapping_node.cpp
#         SP2 --> ODOM[publish_odometry<br/>里程计 + TF变换]
#         SP2 --> PATH[publish_path<br/>运动轨迹]
#         SP2 --> MKR[Visualization_speed<br/>速度向量箭头]
#         FU --> CLD[publish_frame_world<br/>世界系点云]
#         FU --> CLB[publish_frame_body<br/>body系点云]
#     end
#
#     subgraph ROS消息
#         ODOM -->|nav_msgs::Odometry| ROS_O[/Odometry]
#         PATH -->|nav_msgs::Path| ROS_P[/path]
#         CLD -->|PointCloud2| ROS_C[/cloud_registered]
#         CLB -->|PointCloud2| ROS_B[/cloud_registered_body]
#         MKR -->|Marker| ROS_M[/speed_vector]
#         IKDT -->|PointCloud2| ROS_MAP[/Laser_map]
#     end
# ```
#
#
# 五、坐标系变换链
# ================
#
#   LiDAR系                      IMU系(Body系)              世界系(World)
#   ┌─────┐    offset_R_L_I     ┌─────┐       rot, pos      ┌─────┐
#   │  P  │ ──────────────────► │  P  │ ──────────────────► │  P  │
#   │ lid │    offset_T_L_I     │ imu │     P_w = R*P_i+t   │  w  │
#   └─────┘                     └─────┘                     └─────┘
#
#   发布时额外姿态补偿（无人机倾斜安装）:
#         P_pub = R_3 * R_roll * R_pitch * P_w
#         q_pub = R_3 * R_roll * R_pitch * R * (R_3 * R_roll * R_pitch)⁻¹
#         其中 R_3 = diag(1, -1, -1)
#
#
# 六、ESKF 迭代更新公式
# =====================
#
#  预测步骤（IMU前向传播）:
#    x̂ = x ⊞ (dt · f(x,u))           -- 状态传播
#    P̂ = Fₓ · P · Fₓᵀ + G · Q · Gᵀ   -- 协方差传播
#
#  迭代更新步骤（点到平面残差）:
#    for k = 0..max_iter:
#      计算 H_k = dh/dx|x_k           -- 观测雅可比
#      计算 h_k = -点到平面距离        -- 残差
#      K = (HᵀH/R + P̂⁻¹)⁻¹ · Hᵀ/R    -- 卡尔曼增益
#      dx = K·h + (K·H - I)·(x_k ⊟ x̂) -- 状态增量
#      x_{k+1} = x_k ⊞ dx             -- 状态更新
#      if |dx| < ε: break             -- 收敛检查
#    P = (I - K·H) · P̂                -- 协方差更新
#
#  其中 ⊞ 是流形上的广义加法:
#    非旋转量: pos + δp
#    旋转量:   R · exp(δθ∧)
#
#     ⊟ 是流形上的广义减法:
#    非旋转量: pos₁ - pos₂
#    旋转量:   log(R₂ᵀ · R₁)
#
#
# 七、编译依赖关系
# ================
#
#   可执行文件: ralio_mapping
#   源文件:
#     src/laser_mapping.cpp
#     src/preprocess.cpp
#     include/ikd-Tree/ikd_Tree.cpp
#
#   外部依赖:
#     ROS (roscpp, sensor_msgs, nav_msgs, tf, pcl_ros, ...)
#     Eigen3
#     PCL >= 1.8
#     Sophus (SO3 Lie group library)
#     Livox ROS Driver
#     OpenMP (可选, 多线程加速)
#     rosfmt (日志格式化)
