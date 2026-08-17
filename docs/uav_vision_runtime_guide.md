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
   - ID 4 是降落区中央 Tag；出现时直接使用它的二维像素中心。
   - ID 4 不可见但四个角 Tag 可见时，使用投影四边形的对角线交点。
   - 只有三个角 Tag 时，使用最长点对的中点作为二维仿射近似中心。
   - 只有一到两个角 Tag 时，对可见 Tag 中心取平均作为兜底。
   - Tag ID 用于过滤非比赛标签，不要求五个标签同时出现。
   - 对每帧得到的二维中心再进行最近 5 帧中值滤波。
   - 已获得有效结果后允许连续漏检 4 帧，第 5 个连续空检测帧才判定识别失败。
   - 使用 `/mavros/local_position/pose` 的融合姿态和可在线调节的相机安装外参，对像素偏差做姿态虚拟稳像后输出。

两个相机节点通过 `/vision/control` 接收统一的期望状态。正式任务中由
`fsm_ctrl` 的 SML 状态机在每个 Tick 按飞行阶段控制前视和下视相机采集。
目标分类和降落识别节点不再由该消息启停：它们保持运行，收到图像就处理。

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
roslaunch uav_vision dual_camera.launch always_enabled:=true
```

这里是脱离状态机的独立视觉调试流程，因此显式使两个相机始终开启。
随整机任务运行时不要传 `always_enabled` 覆盖参数，由
`/vision/control` 控制相机采集。

该 launch 同时启动：

- `/front_camera_node`
- `/down_camera_node`

### 4.3 启动 AprilTag 降落识别

终端 3：

```bash
roslaunch uav_vision landing_tag.launch
```

该节点始终保持运行；下视相机有图像到达时就执行 AprilTag 识别。

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

该节点同样始终保持运行；前视相机有图像到达时就执行目标匹配。

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
| `/front_camera_node` | `/vision/control` | `/vision/front/image_raw` | 按期望状态幂等地开启或释放前视相机，开启时发布图像 |
| `/down_camera_node` | `/vision/control` | `/vision/down/image_raw` | 按期望状态幂等地开启或释放下视相机，开启时发布图像 |
| `/landing_tag_node` | `/vision/down/image_raw` | `/vision/landing/offset`、`/vision/landing/debug_image` | 收到图像就识别五个 AprilTag 并计算像素偏差 |
| `/target_match_node` | `/vision/front/image_raw` | `/vision/target/result`、`/vision/target/debug_image` | 收到图像就执行 ORB/AKAZE 特征定位、方框兜底和四类别模板匹配 |
| `/landing_debug_web` | `/vision/landing/debug_image` | HTTP 8080 端口 | 在浏览器中显示降落调试画面 |
| `/target_debug_web` | `/vision/target/debug_image` | HTTP 8081 端口 | 在浏览器中显示目标分类调试画面 |

所有图像订阅和发布队列均以低延迟为目标，队列长度为 1。

### 5.1 相机采集启停控制消息

消息类型：

```text
uav_vision_msgs/VisionControl
```

定义文件：

```text
src/uav_vision_msgs/msg/VisionControl.msg
```

原始定义：

```text
std_msgs/Header header
bool front_camera_enabled
bool down_camera_enabled
```

控制话题：

```text
/vision/control
```

字段含义：

| 字段 | 含义 |
|---|---|
| `header` | 当前 SML Tick 发布控制快照的时间；`frame_id` 留空 |
| `front_camera_enabled` | `true` 时保持前视相机采集和图像发布，`false` 时释放前视相机 |
| `down_camera_enabled` | `true` 时保持下视相机采集和图像发布，`false` 时释放下视相机 |

该消息表达“收到后应保持的期望状态”，不是需要翻转本地状态的一次性
start/stop 脉冲。SML 状态机在每个 Tick 发布当前完整快照，正常主循环下约为
50 Hz；即使状态没有切换，同一期望状态也会继续发布。发布端同时保留
latched publisher，作为相机节点晚启动时立即获取最后一份快照的附加保障。
两项布尔值在同一消息内作为一份完整状态更新。

相机节点必须幂等地应用快照：重复收到相同的 `true` 不得重复打开设备，重复
收到相同的 `false` 也不得重复释放。从 `false` 变为 `true` 时，节点打开
`VideoCapture`、重新应用曝光和增益参数，并恢复图像发布；从 `true` 变为
`false` 时，节点调用 `release()` 并停止发布该路图像。这是对 V4L2 采集设备的逻辑
打开和释放，不是对 USB 相机的物理断电。

目标匹配和降落识别节点保持运行，不订阅 `/vision/control`。相机关闭期间没有
新图像，因此算法不会产生新结果；相机再次开启后，节点会对新到达的每帧图像
自动恢复处理。

当前 `ActiveStateMachine` 使用 `SegmentedMissionMachine`，默认映射如下：

| UDP 命令 | SML 状态 | 前视相机 | 下视相机 |
|---:|---|---|---|
| `0` | `Idle` | 关 | 关 |
| `1` | `ArmOnly` | 关 | 关 |
| `2` | `NmpcHover` | 关 | 关 |
| `3` | `SuperSegment1` | 开 | 关 |
| `4` | `SuperSegment2` | 开 | 关 |
| `5` | `SuperSegment3` | 关 | 开 |
| `6` | `Landing` | 关 | 开 |
| `7`、`8`、不支持的命令 | `SafeNoop` | 关 | 关 |
| `9` | `Emergency` | 关 | 关 |

除表中明确开启的状态外，两路相机均关闭。状态机退出前视任务阶段时必须把
`front_camera_enabled` 置为 `false`；退出下视任务阶段时同理，不能依赖接收端
自行推断上一个状态。

两个相机节点的私有参数 `always_enabled` 用于脱离状态机单独调试：

- `always_enabled=false`：服从 `/vision/control`；在尚未收到控制快照时默认关闭。
- `always_enabled=true`：忽略对应的关闭指令，相机始终保持采集和图像发布。

正式任务应使用 `always_enabled=false`。`true` 只用于相机、算法或网页画面的
独立调试，不能用来验证状态机对相机的启停逻辑。

查看当前期望状态和周期发布频率：

```bash
rostopic echo -n 1 /vision/control
rostopic hz /vision/control
```

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
| `header` | 时间戳继承下视图像；完成稳像时 `frame_id` 为 `uav_down_camera_level`，否则保留下视图像坐标系 |
| `valid` | 当前降落偏差是否可用；短时漏检期间可能复用最近一次有效结果 |
| `dx` | 保留图像 X 正负号、消除 roll/pitch 后的水平化偏差，尺度仍为像素 |
| `dy` | 保留图像 Y 正负号、消除 roll/pitch 后的水平化偏差，尺度仍为像素 |
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

- ID `0、1、2、3、4` 中至少一个合格 Tag 出现；不要求五个同时出现。
- 当前出现的同一 ID 不能重复。
- Hamming 距离不能超过配置阈值。
- AprilTag decision margin 不能低于配置阈值。
- 与上一有效中心的跳变不能超过配置阈值。

新识别结果 `valid=true` 时，正常应有：

```text
1 <= tag_count <= 5
tag_ids = 当前参与中心平均的期望 Tag ID
```

已有有效定位后，如果当前帧没有识别到任何期望 Tag，则连续第 1～4 个空检测帧仍
发布 `valid=true`，并复用最近一次滤波后的 `center_x/center_y/dx/dy`。这些保持帧的
`tag_count/tag_ids` 复用最近一次有效检测，调试图状态为 `missing_hold_N_of_5`。连续第 5 帧为空时
才发布 `valid=false`，同时清空旧中心。节点刚启动且从未得到有效定位时，空帧立即
无效，不会生成虚假的保持结果。

`valid=false` 时：

- `dx`、`dy`、`center_x`、`center_y` 为 `NaN`，不可用于控制。
- `tag_count` 和 `tag_ids` 仍用于说明本帧看到了哪些期望 ID。
- 连续空检测第 5 帧清空保持中心；其他质量无效帧连续 3 帧后清空历史滤波状态。

当前姿态稳像变换为：

```text
camera_ray = [pixel_dx / f, pixel_dy / f, 1]
body_ray = R_body_camera * camera_ray
level_body_ray = Rz(-fused_yaw) * R_world_body(q_fused) * body_ray
level_camera_ray = transpose(R_body_camera) * level_body_ray
level_dx = f * level_camera_ray.x / level_camera_ray.z
level_dy = f * level_camera_ray.y / level_camera_ray.z
```

`f` 是 `pose/focal_length_px`，当前用 640 px 作为无正式内参时的近似焦距。
`R_body_camera` 由 `pose/camera_to_body_rpy_deg: [roll, pitch, yaw]` 生成，角度单位为度，
旋转顺序为 `Rz(yaw) * Ry(pitch) * Rx(roll)`；节点每 0.2 秒读取一次，因此修改后无需重启。
现场已确认的正式值为 `[180,0,90]`，即图像 `+X -> 机体 +Y`、图像 `+Y -> 机体 +X`、
光轴 `+Z -> 机体 -Z`。公式中的 `Rz(-fused_yaw)` 会保留飞机当前 yaw：只改变 yaw 时，
水平化 dx/dy 与原始图像 dx/dy 相等。没有收到姿态、姿态超过 0.2 秒未更新或四元数非法
时，节点继续发布原始像素偏差。该结果仍是像素尺度的姿态稳像量，不是米制地面位移，
也不补偿飞机平移。

下视调试图同时显示两条箭头：粗紫色箭头是图像坐标系的原始偏差，细青色箭头是
保留 yaw 的水平化输出；没有可用位姿时，细箭头变为黄色并标记 `RAW fallback`。
左上角还会显示融合姿态 R/P/Y、位姿延迟、输出坐标系数值、当前外参 R/P/Y 和
`R_body_camera` 三行矩阵。网页顶部提供三个滑块和数值框，点击“应用外参”后直接写入
ROS 参数，约 0.2 秒后观察青色箭头的变化；目标是晃动 roll/pitch 时紫色原始箭头可动，
而青色输出尽量保持稳定。

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

相机采集启停统一使用 `/vision/control`，不要再增加彼此独立的 start/stop 服务或
裸整数模式话题。相机节点应直接采用消息中的布尔值作为完整期望状态；若重复
收到相同快照，不得重复打开或释放设备。`always_enabled=true` 是相机节点的本地调试
覆盖项，不改变 `/vision/control` 本身的含义。目标匹配和降落识别算法不解析该消息，
只要收到对应图像就继续处理。

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
rostopic info /vision/control
```

查看自定义消息定义：

```bash
rosmsg show uav_vision_msgs/LandingOffset
rosmsg show uav_vision_msgs/TargetMatchArray
rosmsg show uav_vision_msgs/TargetMatch
rosmsg show uav_vision_msgs/VisionControl
```

检查相机帧率：

```bash
rostopic hz /vision/control
rostopic hz /vision/front/image_raw
rostopic hz /vision/down/image_raw
```

`/vision/control` 在 SML 主循环运行时应约为 50 Hz。某路相机字段为 `false` 时，
对应 `image_raw` 话题不再有新帧；重新置为 `true` 后应恢复约 30 Hz 的图像。

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
