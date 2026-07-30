# 无人机视觉感知节点运行与消息说明

本文档说明当前 `plan_yaw` 分支中视觉感知模块的连接、启动、ROS 节点、话题和自定义消息定义。

## 1. 当前功能

视觉模块包含两条相互独立的处理链：

1. 前视相机目标分类
   - 优先使用 ORB/AKAZE 特征和 RANSAC 单应性恢复整张目标图四角。
   - 特征定位失败时，再使用方形外沿检测作为兜底，避免把车身等内部轮廓当成目标板。
   - 使用传统视觉多模板匹配，在 `plane`、`car`、`ship`、`house` 四类中分类。
   - 使用最近 5 帧投票，至少 3 帧一致后才发布有效类别。
2. 下视相机降落区域识别
   - 识别 `tag36h11` 家族中 ID 为 `0、1、2、3、4` 的五个 AprilTag。
   - Tag ID 只用于确认五个标签都存在，不与空间位置一一对应。
   - 每帧先对五个 Tag 中心取平均，再对最近 5 帧中心做中值滤波。
   - 输出降落区域中心相对图像中心的像素偏差。

当前两个感知节点都是持续运行模式：收到图像就处理并发布结果。任务触发消息尚未定义，也尚未接入。

## 2. SSH 连接与工作空间

NUC 当前连接信息：

- 当前临时 IP：`10.152.160.55`
- 用户名：`flag`
- 密码：`flag`
- ROS 工作空间：`/home/flag/Jiangyin2026`
- Git 分支：`plan_yaw`

在本机终端连接：

```bash
ssh flag@10.152.160.55
```

输入密码 `flag`。本文后续命令按 Bash 编写；如果 SSH 登录后的终端不是 Bash，先执行：

```bash
bash
```

然后进入工作空间并加载 ROS 环境：

```bash
cd /home/flag/Jiangyin2026
source devel/setup.bash
```

每打开一个新的 SSH 终端，都要重新执行上面两条命令。只有修改代码或消息定义后才需要重新编译：

```bash
cd /home/flag/Jiangyin2026
catkin_make
source devel/setup.bash
```

## 3. 相机设备和固定参数

udev 固定设备名：

| 相机 | 设备名 | ROS 图像话题 | `frame_id` |
|---|---|---|---|
| 前视相机 | `/dev/uav_front_camera` | `/vision/front/image_raw` | `uav_front_camera` |
| 下视相机 | `/dev/uav_down_camera` | `/vision/down/image_raw` | `uav_down_camera` |

两个相机共用以下参数：

| 参数 | 当前值 |
|---|---:|
| 分辨率 | 1280 × 720 |
| 像素格式 | MJPG |
| 目标帧率 | 30 Hz |
| 自动曝光 | 关闭 |
| 曝光值 | 150 |
| 增益 | 5 |
| 动态帧率 | 关闭 |

`dual_camera.launch` 还会对前视画面固定旋转 180°，下视画面不旋转。
该处理只改变前视 ROS 图像方向，不改变相机采集分辨率和帧率。

配置文件：

```text
src/uav_vision/config/camera_common.yaml
```

开机后可先检查 udev 设备名是否存在：

```bash
ls -l /dev/uav_front_camera /dev/uav_down_camera
```

## 4. 推荐启动顺序

建议为每组命令单独开一个 SSH 终端。每个新终端先执行：

```bash
cd /home/flag/Jiangyin2026
source devel/setup.bash
```

### 4.1 启动 ROS Master

终端 1：

```bash
roscore
```

### 4.2 启动两个相机

终端 2：

```bash
roslaunch uav_vision dual_camera.launch
```

该 launch 同时启动：

- `/front_camera_node`
- `/down_camera_node`

### 4.3 启动 AprilTag 降落识别

终端 3：

```bash
roslaunch uav_vision landing_tag.launch
```

该 launch 启动：

- `/landing_tag_node`

输入：

- `/vision/down/image_raw`

输出：

- `/vision/landing/offset`
- `/vision/landing/debug_image`

### 4.4 启动前视目标分类

终端 4：

```bash
roslaunch uav_vision target_match.launch
```

该 launch 启动：

- `/target_match_node`

输入：

- `/vision/front/image_raw`

输出：

- `/vision/target/result`
- `/vision/target/debug_image`

模板目录：

```text
src/uav_vision/templates/
├── plane.png
├── car.png
├── ship.png
└── house.png
```

### 4.5 启动浏览器可视化

终端 5：

```bash
roslaunch uav_vision landing_debug_web.launch
```

在与 NUC 同一网络的电脑浏览器中打开：

```text
http://10.152.160.55:8080
```

网页画面包含五个 Tag 的边框、ID、图像中心、降落区域中心和像素偏差。该网页节点只负责显示；关闭它不会影响 `/landing_tag_node` 的识别和结果发布。

前视分类调试图可使用：

```bash
rosrun rqt_image_view rqt_image_view
```

启动后在界面中选择 `/vision/target/debug_image`。

也可以在一个新的终端启动独立的目标分类网页：

```bash
roslaunch uav_vision target_debug_web.launch
```

然后打开：

```text
http://10.152.160.55:8081
```

端口 `8080` 固定用于降落可视化，端口 `8081` 固定用于目标分类
可视化。两个网页节点相互独立，关闭任一网页都不会停止对应感知节点
或结果消息发布。

目标分类网页为降低无线传输延迟，固定使用 960 × 540、JPEG 质量 70、
最大 10 FPS。该限制只作用于 HTTP 网页流；分类节点仍使用
1280 × 720、约 30 Hz 的原始前视图像。

## 5. 节点与话题总表

| 节点 | 订阅 | 发布 | 作用 |
|---|---|---|---|
| `/front_camera_node` | 无 | `/vision/front/image_raw` | 发布前视相机图像 |
| `/down_camera_node` | 无 | `/vision/down/image_raw` | 发布下视相机图像 |
| `/landing_tag_node` | `/vision/down/image_raw` | `/vision/landing/offset`、`/vision/landing/debug_image` | 识别五个 AprilTag 并计算像素偏差 |
| `/target_match_node` | `/vision/front/image_raw` | `/vision/target/result`、`/vision/target/debug_image` | ORB/AKAZE 特征优先定位、方框兜底和四类别模板匹配 |
| `/landing_debug_web` | `/vision/landing/debug_image` | HTTP 8080 端口 | 在浏览器中显示降落调试画面 |
| `/target_debug_web` | `/vision/target/debug_image` | HTTP 8081 端口 | 在浏览器中显示目标分类调试画面 |

所有图像订阅和发布队列均以低延迟为目标，队列长度为 1。

## 6. 降落识别消息

消息类型：

```text
uav_vision_msgs/LandingOffset
```

定义文件：

```text
src/uav_vision_msgs/msg/LandingOffset.msg
```

原始定义：

```text
std_msgs/Header header
bool valid
float32 dx
float32 dy
float32 center_x
float32 center_y
uint8 tag_count
int32[] tag_ids
```

字段含义：

| 字段 | 含义 |
|---|---|
| `header` | 直接继承下视相机图像的时间戳和 `frame_id` |
| `valid` | 本帧降落偏差是否可用；上层控制必须先判断该字段 |
| `dx` | 降落中心相对图像中心的横向像素差，向右为正、向左为负 |
| `dy` | 降落中心相对图像中心的纵向像素差，向下为正、向上为负 |
| `center_x` | 滤波后的降落中心横坐标，单位为像素 |
| `center_y` | 滤波后的降落中心纵坐标，单位为像素 |
| `tag_count` | 当前识别到的期望 Tag ID 数量 |
| `tag_ids` | 当前识别到的期望 Tag ID，当前期望集合为 `[0, 1, 2, 3, 4]` |

在 1280 × 720 图像中：

```text
image_center_x = 640
image_center_y = 360
dx = center_x - 640
dy = center_y - 360
```

有效条件：

- ID `0、1、2、3、4` 必须全部且各出现一次。
- Hamming 距离不能超过配置阈值。
- AprilTag decision margin 不能低于配置阈值。
- 与上一有效中心的跳变不能超过配置阈值。

`valid=true` 时，正常应有：

```text
tag_count = 5
tag_ids = [0, 1, 2, 3, 4]
```

`valid=false` 时：

- `dx`、`dy`、`center_x`、`center_y` 为 `NaN`，不可用于控制。
- `tag_count` 和 `tag_ids` 仍用于说明本帧看到了哪些期望 ID。
- 连续 3 帧无效后清空历史滤波状态。

注意：输出是图像像素偏差，不是米、厘米、角度或机体系坐标，因此不需要相机标定和 Tag 实际边长。

查看消息：

```bash
rostopic echo /vision/landing/offset
rostopic echo -n 1 /vision/landing/offset
rostopic hz /vision/landing/offset
```

## 7. 目标分类消息

分类结果由一个数组消息和一个单目标消息组成。

### 7.1 `TargetMatchArray`

消息类型：

```text
uav_vision_msgs/TargetMatchArray
```

定义：

```text
std_msgs/Header header
bool valid
uav_vision_msgs/TargetMatch[] matches
```

字段含义：

| 字段 | 含义 |
|---|---|
| `header` | 直接继承前视相机图像的时间戳和 `frame_id` |
| `valid` | 本帧是否具有经过时间投票确认的有效分类结果 |
| `matches` | 有效目标结果数组；当前实现有效时只有 1 项，无效时为空 |

当前时间投票窗口为 5 帧，至少 3 帧同类才稳定。类别稳定后，允许
跨过最多 2 个连续空帧：这两帧继续发布最近一次稳定结果；连续第 3
帧丢失时清空状态并发布 `valid=false`。若检测到另一个有效类别，
不会沿用旧类别，而是重新投票。

跨过空帧时，`header` 使用当前前视图像的时间戳，但 `matches[0]`
复用最近一次稳定结果，因此其中的 `corners` 和分数最多可能滞后 2
帧。上层任务只应使用稳定后的 `label` 做类别判断，不应把该消息的
角点用于实时控制。

### 7.2 `TargetMatch`

消息类型：

```text
uav_vision_msgs/TargetMatch
```

定义：

```text
string label
float32 score
float32 gray_score
float32 hog_score
float32 color_score
float32 margin
float32 sharpness
uint16 target_side_px
float32[8] corners
```

字段含义：

| 字段 | 含义 |
|---|---|
| `label` | 分类标签：`plane`、`car`、`ship` 或 `house` |
| `score` | 最佳类别的融合匹配分数，范围约为 0～1，越大越相似 |
| `gray_score` | 灰度模板相关性分数，范围 0～1 |
| `hog_score` | HOG 形状特征相似度，范围 0～1 |
| `color_score` | HSV 颜色直方图相关性分数，范围 0～1 |
| `margin` | 最佳类别分数减去第二类别分数，越大表示类别区分越明确 |
| `sharpness` | 目标区域的拉普拉斯方差，用于判断模糊程度；它不是 0～1 归一化分数 |
| `target_side_px` | 检测到的方形目标平均边长，单位为像素 |
| `corners` | 方形目标四角坐标，格式见下方 |

当前融合分数权重：

```text
score = 0.50 × gray_score
      + 0.30 × hog_score
      + 0.20 × color_score
```

`corners` 数组顺序：

```text
[左上x, 左上y,
 右上x, 右上y,
 右下x, 右下y,
 左下x, 左下y]
```

查看消息：

```bash
rostopic echo /vision/target/result
rostopic echo -n 1 /vision/target/result
rostopic hz /vision/target/result
```

## 8. 上层程序的使用约定

上层飞控或状态机应遵循以下顺序：

1. 订阅对应结果话题。
2. 首先检查消息的 `valid`。
3. 只有 `valid=true` 时才读取有效数据。
4. 同时检查 `header.stamp`，避免使用过期结果。

降落控制伪代码：

```text
收到 /vision/landing/offset:
    if valid:
        使用 dx、dy 做图像平面闭环控制
    else:
        不使用 dx、dy，执行悬停、重试或任务状态机设定的失效策略
```

目标分类伪代码：

```text
收到 /vision/target/result:
    if valid and matches 非空:
        使用 matches[0].label
    else:
        视为当前没有可靠分类结果
```

当前尚未确定任务触发消息，所以不要假设存在 start/stop 服务或触发话题。后续接入时，可保留本文件所述结果消息不变，只在感知节点前增加启停状态控制。

## 9. 常用检查命令

查看节点：

```bash
rosnode list
```

查看视觉话题：

```bash
rostopic list | grep '^/vision/'
```

检查话题类型和连接：

```bash
rostopic info /vision/landing/offset
rostopic info /vision/target/result
```

查看自定义消息定义：

```bash
rosmsg show uav_vision_msgs/LandingOffset
rosmsg show uav_vision_msgs/TargetMatchArray
rosmsg show uav_vision_msgs/TargetMatch
```

检查相机帧率：

```bash
rostopic hz /vision/front/image_raw
rostopic hz /vision/down/image_raw
```

检查设备是否被进程占用：

```bash
fuser /dev/uav_front_camera
fuser /dev/uav_down_camera
```

## 10. 停止节点

在对应启动终端按 `Ctrl+C`。建议按以下顺序关闭：

1. `target_debug_web.launch`
2. `landing_debug_web.launch`
3. `target_match.launch`
4. `landing_tag.launch`
5. `dual_camera.launch`
6. `roscore`

若只关闭浏览器可视化节点，对应识别结果仍会继续发布。

## 11. 当前实机验证记录

在固定参数“曝光 150、增益 5、关闭动态帧率”下已验证：

- 前视相机约 30 Hz。
- 下视相机约 30 Hz。
- AprilTag 结果消息连续 150 帧均为 `valid=true`。
- 150 帧中 `tag_count` 均为 5，`tag_ids` 均为 `[0, 1, 2, 3, 4]`。
- `/vision/landing/offset` 实测约 26.9 Hz；低于相机 30 Hz 是 AprilTag 处理开销造成的正常现象。
- 消息时间戳严格递增，偏差公式和字段一致性检查通过。
- 房子打印图在外沿对比不足时，ORB 兜底连续 36 帧均识别为
  `house` 且 `valid=true`，分类结果约 12 Hz。
- 汽车打印图近距离验证中，特征优先流程连续 102 条消息均为
  `car` 且 `valid=true`，未出现 `house` 或 `unknown`，约 12.2 Hz。

以上验证只代表当时道具位置和光照条件。重新搭建场地后，仍应先用浏览器调试画面确认五个 Tag 都清晰、无遮挡。
