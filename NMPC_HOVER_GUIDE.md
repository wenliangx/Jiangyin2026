# NMPC 悬停测试指南 (Mocap 模式)

> **适用场景：** 使用动作捕捉系统（OptiTrack/Motive）提供位姿，通过 NMPC 控制器实现无人机悬停。
>
> **硬件要求：** PX4飞控、机载计算机、OptiTrack动捕系统、遥控器（可选）
>
> **作者：** FLAG Lab, BIT
>
> **日期：** 2025-06-08

---

## 目录

1. [前置准备](#1-前置准备)
2. [系统架构与数据流](#2-系统架构与数据流)
3. [启动步骤](#3-启动步骤)
4. [NMPC悬停测试操作](#4-nmpc悬停测试操作)
5. [NMPC参数详解](#5-nmpc参数详解)
6. [常见问题排查](#6-常见问题排查)

---

## 1. 前置准备

### 1.1 硬件连接确认

| 连接 | 端口 | 说明 |
|------|------|------|
| 飞控 ↔ 机载计算机 | `/dev/ttyACM0` | USB串口，波特率 921600 |
| 遥控器接收机 ↔ 飞控 | SBUS/DSM | 通道8用于起飞确认 |
| OptiTrack ↔ 交换机 | 网线 | Motive 软件运行在 Windows 上 |
| 机载计算机 ↔ 局域网 | WiFi/网线 | 需与 Motive 主机在同一网络 |

### 1.2 软件环境

```bash
# 确认工作空间路径
cd /home/flag/Jiangyin2025

# 确认已编译
source devel/setup.bash
ls devel/lib/fsm_ctrl/
# 应看到以下可执行文件：
#   px4_estimator          ← 位姿估计桥接节点
#   single_offboard_fsm    ← 主控制FSM节点
#   swarm_user_cmd         ← 用户命令交互节点

# 确认 MAVROS 已安装
rospack find mavros

# 确认 CasADi 已安装（NMPC依赖）
ldconfig -p | grep casadi
```

### 1.3 如需重新编译

```bash
cd /home/flag/Jiangyin2025
catkin_make -j$(nproc)
source devel/setup.bash
```

### 1.4 PX4 飞控参数确认

在 QGroundControl 中确认以下 EKF2 参数已正确设置：

| 参数 | 值 | 说明 |
|------|-----|------|
| `EKF2_AID_MASK` | 1 (vision position) 或 3 (vision position + yaw) | 启用视觉位姿融合 |
| `EKF2_EV_DELAY` | 0 | 视觉数据延迟（0=无延迟） |
| `EKF2_HGT_MODE` | 0 或 3 | 高度来源（0=气压计, 3=视觉） |
| `MAV_1_CONFIG` | 0 (Disabled) 或 TELEM1 | 确认 MAVLink 串口配置 |
| `SYS_AUTOSTART` | 4001 | 通用四旋翼机架 |
| `COM_RCL_EXCEPT` | 4 | 允许无遥控器解锁（仅测试用） |

---

## 2. 系统架构与数据流

### 2.1 完整数据流图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        OptiTrack 动捕系统                            │
│                    (Motive 软件运行在 Windows)                        │
└────────────────────────────┬────────────────────────────────────────┘
                             │ VRPN 协议 (UDP)
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  vrpn_client_node (ROS节点)                                          │
│  发布话题: /vrpn_client_node/jy0/pose (geometry_msgs/PoseStamped)    │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  px4_estimator 节点                                                  │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ vision_source = 0 (mocap模式)                                  │   │
│  │                                                                │   │
│  │ 1. 读取首帧作为坐标原点 (pos_init, quat_init)                  │   │
│  │ 2. 后续所有帧进行偏移变换:                                     │   │
│  │    pos = quat_init⁻¹ · (pos_raw - pos_init)                    │   │
│  │    quat = quat_init⁻¹ · quat_raw                               │   │
│  │ 3. 发布 /mavros/vision_pose/pose (100Hz)                      │   │
│  │ 4. 比对 vision pose 与 FCU EKF 融合结果，                      │   │
│  │    收敛后发布 /fsm_ctrl/ekf_ready = true                       │   │
│  └──────────────────────────────────────────────────────────────┘   │
└────────────────────────────┬────────────────────────────────────────┘
                             │ /mavros/vision_pose/pose
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  MAVROS (mavros/launch/px4.launch)                                   │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ 通过 /dev/ttyACM0:921600 与飞控通信                            │   │
│  │ 上行: vision_pose/pose → PX4 EKF2                             │   │
│  │ 下行: local_position/pose ← PX4 EKF2 (200Hz)                  │   │
│  │      local_position/velocity_local ← PX4 EKF2                 │   │
│  │      rc/in ← 遥控器信号                                        │   │
│  │      state ← 飞控状态 (armed, mode等)                         │   │
│  │      imu/data ← IMU数据                                       │   │
│  │ 下行→飞控: setpoint_raw/attitude (bodyrate + thrust)          │   │
│  └──────────────────────────────────────────────────────────────┘   │
└────────────────────────────┬────────────────────────────────────────┘
                             │ /mavros/local_position/pose
                             │ /mavros/local_position/velocity_local
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  single_offboard_fsm 节点 (主控制器，50Hz)                            │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ UDP 端口 12001 监听命令                                        │   │
│  │                                                                │   │
│  │ cmd=1 (Arm):    切换到OFFBOARD模式 + 解锁 + 怠速推力0.02       │   │
│  │ cmd=3 (Takeoff): RC通道8>1500确认后，推力0.05起飞             │   │
│  │ cmd=5 (NMPC):   运行NMPC简单控制器，悬停在(0,0,0.5)            │   │
│  │ cmd=4 (Land):   降落至0.005m后自动上锁                         │   │
│  │                                                                │   │
│  │ NMPC控制器（cmd=5时运行）：                                    │   │
│  │   - 10状态: px,py,pz, vx,vy,vz, qw,qx,qy,qz                  │   │
│  │   - 4输入:  wx,wy,wz(机体角速度), acc_z(机体Z轴加速度)        │   │
│  │   - 预测步长: 8步, 每步0.05s, 总预测0.4s                      │   │
│  │   - 求解器: CasADi NLP                                         │   │
│  │   - 输出: /mavros/setpoint_raw/attitude (IGNORE_ATTITUDE)     │   │
│  └──────────────────────────────────────────────────────────────┘   │
└────────────────────────────┬────────────────────────────────────────┘
                             │ /mavros/setpoint_raw/attitude
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  MAVROS → PX4飞控 → 电调 → 电机                                     │
│  (PX4内部姿态控制器接收 bodyrate+thrust 指令，驱动电机)             │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 NMPC控制器详解

**状态方程（10维 → 10维导数）：**

```
ṗ    = v
v̇_x  = 2(q₀q₂ + q₁q₃) · acc_z
v̇_y  = 2(q₂q₃ - q₀q₁) · acc_z
v̇_z  = (q₀² - q₁² - q₂² + q₃²) · acc_z - g
q̇₀   = 0.5(-q₁ωₓ - q₂ω_y - q₃ω_z)
q̇₁   = 0.5(q₀ωₓ + q₂ω_z - q₃ω_y)
q̇₂   = 0.5(q₀ω_y - q₁ω_z + q₃ωₓ)
q̇₃   = 0.5(q₀ω_z + q₁ω_y - q₂ωₓ)
```

其中 `g = 9.8015 m/s²`，模型假设无人机质量 `m = 1.874 kg`。

**输入约束：**
- 角速度 ω: [-3.14, 3.14] rad/s
- Z轴加速度 acc_z: [0, 15] m/s²

**悬停时 NMPC 期望状态（由 cmd=5 设定）：**

```
位置: (0, 0, 0.5) m    ← 以起飞点为原点，高度0.5m悬停
速度: (0, 0, 0) m/s
姿态: 四元数 (1, 0, 0, 0) ← 水平姿态
```

---

## 3. 启动步骤

### 3.0 启动前检查

```bash
# 1. 确认 Motive 软件正在运行，OptiTrack 相机已校准
# 2. 确认无人机上电，飞控启动完成（LED常亮/特定闪烁模式）
# 3. 确认机载计算机与飞控的USB线已连接
# 4. 在机载计算机上检查串口
ls /dev/ttyACM* /dev/ttyUSB*
# 应看到 /dev/ttyACM0 或类似设备

# 5. 检查网络连通性（能ping通Motive主机）
# 6. 确认所有ROS依赖已安装
```

### 3.1 终端布局建议

建议开 **4个终端**，使用 `tmux` 或手动开4个终端窗口：

```
┌──────────────┬──────────────────────┐
│  终端1       │  终端2               │
│  roscore     │  VRPN客户端          │
│              │  (动捕数据)          │
├──────────────┼──────────────────────┤
│  终端3       │  终端4               │
│  single.launch│  swarm.launch       │
│  (MAVROS+FSM)│  (estimator+cmd_ui) │
└──────────────┴──────────────────────┘
```

### 3.2 终端1：启动 ROS Master

```bash
cd /home/flag/Jiangyin2025
source devel/setup.bash
roscore
```

**预期输出：** `started core service [/rosout]`

### 3.3 终端2：启动 VRPN 客户端（动捕数据）

> ⚠️ **必须先确认 Motive 软件已开始广播 VRPN 数据**

```bash
cd /home/flag/Jiangyin2025
source devel/setup.bash

# 方式一：如果安装了 vrpn_client_ros 包
roslaunch vrpn_client_ros sample.launch server:=<Motive_IP地址>

# 方式二：直接运行节点
rosrun vrpn_client_ros vrpn_client_node _server:=<Motive_IP地址>

# 例如 Motive 运行在 192.168.1.100
# rosrun vrpn_client_ros vrpn_client_node _server:=192.168.1.100
```

**验证动捕数据是否到达：**

```bash
# 新开一个终端或使用 rostopic echo
rostopic echo /vrpn_client_node/jy0/pose -n 1
```

应该看到类似输出：
```
header:
  seq: 1234
  stamp: ...
  frame_id: "world"
pose:
  position:
    x: 0.123
    y: -0.456
    z: 1.789
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

> ⚠️ **如果收不到数据**：检查 Motive 中刚体名称是否为 `jy0`（与 `px4_estimator.cpp:195` 中硬编码的主题名一致）。如果不一致，需要修改代码或使用 topic relay。

### 3.4 终端3：启动 MAVROS + 主控制器

```bash
cd /home/flag/Jiangyin2025
source devel/setup.bash

# 如果飞控串口不是 /dev/ttyACM0，先修改 launch 文件
# vim src/fsm_ctrl/fsm_ctrl/launch/single.launch  # 修改 fcu_url

roslaunch fsm_ctrl single.launch
```

**该命令启动两个组件：**

| 组件 | 说明 |
|------|------|
| `mavros/launch/px4.launch` | MAVROS节点，`/dev/ttyACM0:921600` |
| `single_offboard_fsm` | 主控制FSM节点，加载NMPC参数 |

**预期输出关键行：**

```
[INFO] MAVROS started
[INFO] FCU: ... connected
[INFO] single_offboard_fsm: started
[INFO] NMPC solver initialized
```

**验证 MAVROS 连接正常：**

```bash
# 另开终端检查
rostopic echo /mavros/state -n 1
# connected 应为 True
# armed 应为 False
# mode 应为 "MANUAL" 或其他非OFFBOARD模式
```

### 3.5 终端4：启动位姿估计器 + 用户命令界面

```bash
cd /home/flag/Jiangyin2025
source devel/setup.bash
roslaunch fsm_ctrl swarm.launch
```

> ⚠️ **重要：** 终端4必须能接收键盘输入，因为 `swarm_user_cmd` 从这里读取命令。

**该命令启动两个组件：**

| 组件 | 说明 |
|------|------|
| `px4_estimator` | vision_source=0 (mocap)，读取动捕位姿→发布vision_pose |
| `swarm_user_cmd` | 终端UI，通过UDP 127.0.0.1:12001发送命令 |

**终端4的界面显示：**

```
>>>>>>>>>>>>>>>>>>>>>>>>>>>- Pose Info -<<<<<<<<<<<<<<<<<<<<<<<<<<<
[FCU] x: +0.000(m)   y: +0.000(m)   z: +0.000(m)   yaw: +0.000(deg)
[VIO] x: +0.000(m)   y: +0.000(m)   z: +0.000(m)   yaw: +0.000(deg)
>>>>>>>>>>>>>>>>>>>>>>>>>>>- User Info -<<<<<<<<<<<<<<<<<<<<<<<<<<<
1: Arm         2: Disarm      3: Takeoff     4: Land        5: Debug
6:             7:             8:             9:             0: Exit
Enter User Command:
```

| 显示项 | 含义 | 数据来源 |
|--------|------|---------|
| `[FCU]` | PX4 EKF融合后的位姿 | `/mavros/local_position/pose` |
| `[VIO]` | px4_estimator发布的原始视觉位姿 | `/mavros/vision_pose/pose` |

> **关键验证：** `[FCU]` 和 `[VIO]` 的数值应该非常接近（位置差<0.05m，偏航差<9°），这表示 EKF 融合已收敛。如果不收敛（数值差异大），等待几秒让 EKF 收敛。

### 3.6 可选：启动 rqt 监控

```bash
# 在新终端中监控关键话题
rqt_plot /mavros/local_position/pose/pose/position/z    # 高度
rqt_plot /mavros/setpoint_raw/attitude/thrust            # 油门指令
rqt_plot /mavros/local_position/velocity_local/twist/linear/x:y:z  # 速度
```

---

## 4. NMPC悬停测试操作

### 4.1 操作流程总览

```
cmd=1 (Arm)       cmd=3 (Takeoff)     cmd=5 (NMPC Hover)
  解锁+OFFBOARD  →  推力起飞       →  NMPC控制在(0,0,0.5)悬停
                    (需RC ch8>1500)
```

### 4.2 详细操作步骤

#### 步骤1：Arm — 解锁并进入OFFBOARD模式

在 **终端4** 中输入 `1` 然后按回车。

```
Enter User Command: 1
```

**系统行为：**
1. `single_offboard_fsm` 通过 MAVROS 请求飞控切换到 `OFFBOARD` 模式
2. 等待 OFFBOARD 模式切换成功后，发送解锁命令
3. 发布 `setpoint_raw/attitude`：body_rate=(0,0,0)，thrust=0.02（怠速，不足以起飞）
4. 以 50Hz 不断发布，保持 OFFBOARD 心跳

**预期反应：**
- 飞控 LED 从常亮变为特定闪烁（表示已解锁）
- 电机以怠速旋转（桨叶微微转动）
- 终端4显示 `[CMD] 1`

**验证：**
```bash
rostopic echo /mavros/state -n 1
# armed: True
# mode: "OFFBOARD"
```

#### 步骤2：Takeoff — 推力起飞

在 **终端4** 中输入 `3` 然后按回车。

```
Enter User Command: 3
```

**系统行为（分两种情况）：**

| 情况 | 条件 | 控制方式 |
|------|------|---------|
| A (推荐) | RC通道8 > 1500 | 直接发布 attitude target: thrust=0.05, 水平姿态 |
| B | RC通道8 ≤ 1500 或无遥控器 | 发布 position setpoint: 目标(0, 0, 0.4)，走PX4内部位置控制 |

- **情况A**：无人机以固定推力 0.05 离地（小油门，缓慢上升），之后切换到 cmd=5 的 NMPC 控制
- **情况B**：无人机由 PX4 内部位置控制器飞到 0.4m 高度（非 NMPC）

> **⚠️ 重要：** 本指南专注于 NMPC 控制，强烈推荐使用情况A（有遥控器，RC通道8 > 1500）。如果没有遥控器，见 [4.5节](#45-无遥控器的替代方案)。

**预期反应：**
- 无人机缓慢离地起飞
- 由于推力仅 0.05（略高于悬停油门 0.385 的 13%），上升会非常缓慢
- 终端4显示 `[CMD] 3`

#### 步骤3：NMPC Hover — NMPC悬停

在 **终端4** 中输入 `5` 然后按回车。

```
Enter User Command: 5
```

**系统行为：**
1. FSM 读取当前 FCU 融合位姿作为 `current_states`
2. 构建 NMPC 期望轨迹：9个预测步（8+1），每步目标均为 `(0, 0, 0.5)` 位置、零速度、水平姿态
3. 调用 `nmpc_ctrl_simple.optimal_solution(current_states, desired_states)` 求解最优控制量
4. 将求解结果（机体角速度 ωx,ωy,ωz 和 机体Z轴加速度 acc_z）发布到 `/mavros/setpoint_raw/attitude`
5. PX4 内部姿态控制器跟踪这些 bodyrate 指令，驱动电机

**预期反应：**
- 无人机从当前位置向 (0, 0, 0.5) 移动并悬停
- 如果从起飞直接切换，可能有一个上冲，然后稳定在 0.5m
- NMPC 求解时间约 12ms 左右（在终端中会打印超过12ms的警告）
- 终端4显示 `[CMD] 5`

**NMPC 输出监控：**
```bash
rostopic echo /nmpc_state -n 1
# 查看 target.body_rate 和 target.thrust
# 也查看 pos_ref 和 pos_fdb 的对比
```

#### 步骤4：Land — 降落

在 **终端4** 中输入 `4` 然后按回车。

```
Enter User Command: 4
```

**系统行为：**
- 使用 DFBC（位置控制）降到高度 0.005m
- 到达后自动 disarm（上锁）

#### 步骤5：退出程序

在 **终端4** 中输入 `0` 然后按回车。

```
Enter User Command: 0
```

### 4.3 紧急情况处理

#### 紧急悬停/缓降 (cmd=9)

在 **终端4** 中输入 `9` 然后按回车。

**行为：** 发布 `thrust = nmpc_hover_thrust - 0.03 = 0.385 - 0.03 = 0.355`，略低于悬停油门，实现缓降。

#### 遥控器紧急接管

切换遥控器上的飞行模式开关（从 OFFBOARD 切到 Position/Altitude/Stabilized 等模式），飞控将退出 OFFBOARD 模式，由遥控器手动控制。

#### 紧急停桨

在 QGroundControl 或遥控器上执行紧急停桨命令（具体取决于飞控设置）。

### 4.4 完整操作速查表

| 步骤 | 终端 | 输入 | 说明 |
|------|------|------|------|
| 1 | 终端1 | - | 启动 `roscore` |
| 2 | 终端2 | - | 启动 VRPN 客户端 |
| 3 | 终端3 | - | 启动 `single.launch` (MAVROS+FSM) |
| 4 | 终端4 | - | 启动 `swarm.launch` (estimator+cmd_ui) |
| 5 | 终端4 | `1` | **Arm** — 解锁+OFFBOARD模式 |
| 6 | 终端4 | `3` | **Takeoff** — 推力起飞 |
| 7 | 终端4 | `5` | **NMPC Hover** — NMPC悬停(0,0,0.5) |
| 8 | 终端4 | `4` | **Land** — 降落+上锁 |
| 9 | 终端4 | `0` | 退出 swarm_user_cmd |

### 4.5 无遥控器的替代方案

如果没有遥控器，`takeoff_channel` 将始终为 0，cmd=3 会走情况B（DFBC位置控制）。有以下替代方案：

#### 方案一：修改 cmd=5 使其自带 Arm+起飞功能

修改 `single_offboard_fsm.cpp` 中 cmd=5 块，在最前面添加 Arm 和起飞逻辑。不推荐修改核心代码。

#### 方案二：使用 cmd=1→cmd=5 直接过渡

```
cmd=1 (Arm+OFFBOARD, 怠速0.02) → cmd=5 (NMPC直接从地面控到0.5m)
```

跳过 cmd=3，NMPC 直接从地面控制到目标高度。**风险：** NMPC 可能会输出较大的 thrust 来快速爬升，导致起飞过猛。

#### 方案三：修改 takeoff 的 RC 判断逻辑（推荐）

在 `single_offboard_fsm.cpp:1787` 行，将：
```cpp
if(takeoff_channel > 1500)
```
临时改为：
```cpp
if(true) // 绕过RC检查
```
重新编译后，cmd=3 将直接使用推力起飞模式（thrust=0.05），然后再切换到 cmd=5。

```bash
# 修改后重新编译
cd /home/flag/Jiangyin2025
catkin_make -j$(nproc)
source devel/setup.bash
```

---

## 5. NMPC参数详解

### 5.1 控制器结构参数

在 `single_offboard_fsm.cpp:1629-1639` 中定义：

| 参数 | 值 | 说明 |
|------|-----|------|
| `nmpc_simple_predict_step` | 8 | NMPC 预测步数 |
| `nmpc_simple_sample_time` | 0.05 s | 每步时长，总预测时域 = 8×0.05 = 0.4s |
| INTERV (控制周期) | 0.02 s (50Hz) | 控制器运行频率 |
| `_w_limit` | [-3.14, 3.14] rad/s | 角速度约束 |
| `_acc_z_limit` | [0, 15] m/s² | Z轴加速度约束 |
| `drone_mass` | 1.874 kg | 无人机质量（在 NMPC_Controller.hpp:19） |
| `drone_arm_L` | 0.125 m | 机臂长度 |
| `GRAVITY` | 9.8015 m/s² | 重力加速度 |

### 5.2 NMPC 代价函数权重（simple NMPC，cmd=5使用）

在 `single.launch:96-108` 中配置：

```xml
<!-- 位置权重 -->
<param name="nmpc_Qposx" value="20.0" />
<param name="nmpc_Qposy" value="20.0" />
<param name="nmpc_Qposz" value="50.0" />   ← Z轴权重最大，优先保持高度

<!-- 速度权重 -->
<param name="nmpc_Qvelx" value="1.0" />
<param name="nmpc_Qvely" value="1.0" />
<param name="nmpc_Qvelz" value="1.0" />

<!-- 姿态四元数权重 -->
<param name="nmpc_Qquatx" value="1.0" />
<param name="nmpc_Qquaty" value="1.0" />
<param name="nmpc_Qquatz" value="1.0" />

<!-- 输入代价（角速度） -->
<param name="nmpc_Rwx" value="0.1" />
<param name="nmpc_Rwy" value="0.1" />
<param name="nmpc_Rwz" value="0.1" />

<!-- 输入代价（推力加速度） -->
<param name="nmpc_RtotalF" value="0.1" />
```

### 5.3 调参指南

| 现象 | 调整方向 |
|------|---------|
| 悬停时高度震荡 | 增大 `nmpc_Qposz`（加强高度约束），减小 `nmpc_RtotalF` |
| 悬停时水平漂移 | 增大 `nmpc_Qposx` 和 `nmpc_Qposy` |
| 控制响应太慢 | 增大位置权重，减小速度权重 |
| 角速度指令震荡 | 增大 `nmpc_Rwx/y/z`，或减小 `nmpc_Qquat` |
| 推力输出抖动 | 增大 `nmpc_RtotalF` |
| NMPC 求解超时（>12ms） | 减少 `nmpc_simple_predict_step` 或增大 `nmpc_simple_sample_time` |

### 5.4 悬停目标修改

如需修改 NMPC 悬停的目标位置，编辑 `single_offboard_fsm.cpp:1866-1879`：

```cpp
// 当前悬停目标：(0, 0, 0.5)
desired_states.push_back(0.00);   // ← 修改为期望 X
desired_states.push_back(0.00);   // ← 修改为期望 Y
desired_states.push_back(0.5);    // ← 修改为期望 Z
desired_states.push_back(0.0);    // 期望 Vx
desired_states.push_back(0.0);    // 期望 Vy
desired_states.push_back(0.0);    // 期望 Vz
desired_states.push_back(1.0);    // 四元数 w
desired_states.push_back(0.0);    // 四元数 x
desired_states.push_back(0.0);    // 四元数 y
desired_states.push_back(0.0);    // 四元数 z
```

---

## 6. 常见问题排查

### 6.1 动捕数据相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `/vrpn_client_node/jy0/pose` 无数据 | Motive 未广播或刚体名称不对 | 检查 Motive 中刚体名称是否为 `jy0` |
| `[VIO]` 显示数值异常 | 坐标系不匹配 | 检查 `flag_mocap_frame`（0=Z-up, 1=Y-up），修改 `px4_estimator.cpp:22` |
| `[FCU]` 和 `[VIO]` 差距一直很大 | EKF 不收敛 | 等待更长时间，或检查 EKF2 参数 |

### 6.2 MAVROS/飞控相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `[FATAL] FCU: device closed` | 串口权限/连接问题 | `sudo chmod 777 /dev/ttyACM0` 或加入 dialout 组 |
| `connected: False` | 飞控未启动或串口错误 | 检查飞控上电，确认串口设备名 |
| OFFBOARD 模式切换失败 | 飞控参数不允许 OFFBOARD | 检查 `SYS_AUTOSTART` 参数 |
| 解锁失败 | 飞控拒绝解锁（安全检查） | 在 QGC 中查看拒绝原因；确认 `COM_RCL_EXCEPT`=4（无遥控器时） |

### 6.3 NMPC相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `NMPC used time: xx.x ms` 频繁出现 | 求解器启动初期计算量大 | 正常现象，首次求解后应稳定在10ms左右 |
| NMPC 求解失败/输出NaN | CasADi 求解器发散 | 检查输入状态是否合法（四元数归一化等） |
| 悬停时无人机漂移/不稳定 | 模型参数不匹配 | 调整质量 `drone_mass` 和推力估算参数 |

### 6.4 编译相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `Could not find casadi` | CasADi 未安装 | 安装 CasADi: `sudo apt install ros-noetic-casadi` 或源码编译 |
| `rosparam load traj_waypoints.yaml failed` | config 目录不存在 | **仅警告，不影响运行**。该文件被 `single.launch` 引用但实际 `EgoGeneTraj()` 使用硬编码航点 |
| `find_package(mavros) failed` | mavros 未安装 | `sudo apt install ros-noetic-mavros ros-noetic-mavros-extras` |

### 6.5 快速诊断命令集

```bash
# 检查所有节点是否正常运行
rosnode list
# 应看到: /px4_estimator, /single_offboard_fsm, /swarm_user_cmd, /mavros

# 检查话题发布频率
rostopic hz /mavros/vision_pose/pose        # 应为 ~100Hz
rostopic hz /mavros/local_position/pose      # 应为 ~200Hz
rostopic hz /mavros/setpoint_raw/attitude    # 应为 ~50Hz

# 检查 EKF 收敛状态
rostopic echo /fsm_ctrl/ekf_ready -n 1
# data: True  ← 表示已收敛

# 检查当前飞行模式和解锁状态
rostopic echo /mavros/state -n 1
# armed: True/False
# mode: "OFFBOARD"/"MANUAL"/...

# 查看 NMPC 输出
rostopic echo /nmpc_state -n 1
```

---

## 附录A：一键启动脚本

将以下内容保存为 `~/nmpc_hover_test.sh`：

```bash
#!/bin/bash
# NMPC 悬停测试一键启动脚本

SESSION="nmpc_test"
WS="/home/flag/Jiangyin2025"

# 杀掉旧会话
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -s $SESSION -n main -d
tmux set-option -g mouse on

# 窗口布局：2x2
tmux split-window -h -t $SESSION:main
tmux split-window -v -t $SESSION:main.0
tmux split-window -v -t $SESSION:main.2

# 窗格0: roscore
tmux select-pane -t $SESSION:main.0
tmux send-keys "cd $WS && source devel/setup.bash && roscore" C-m

# 窗格1: VRPN (需替换MOTIVE_IP)
tmux select-pane -t $SESSION:main.1
tmux send-keys "cd $WS && source devel/setup.bash" C-m
tmux send-keys "echo '请手动启动VRPN客户端: rosrun vrpn_client_ros vrpn_client_node _server:=<MOTIVE_IP>'" C-m

# 窗格2: single.launch (MAVROS + FSM)
tmux select-pane -t $SESSION:main.2
tmux send-keys "sleep 3 && cd $WS && source devel/setup.bash" C-m
tmux send-keys "roslaunch fsm_ctrl single.launch" C-m

# 窗格3: swarm.launch (estimator + cmd_ui)
tmux select-pane -t $SESSION:main.3
tmux send-keys "sleep 5 && cd $WS && source devel/setup.bash" C-m
tmux send-keys "roslaunch fsm_ctrl swarm.launch" C-m

tmux attach-session -t $SESSION
```

使用方法：
```bash
chmod +x ~/nmpc_hover_test.sh
~/nmpc_hover_test.sh
```

---

## 附录B：节点/话题/参数速查

### 节点列表

| 节点名 | 可执行文件 | 功能 |
|--------|-----------|------|
| `px4_estimator` | `fsm_ctrl/px4_estimator` | Mocap/Lidar/Camera位姿桥接→vision_pose |
| `single_offboard_fsm` | `fsm_ctrl/single_offboard_fsm` | 主控FSM + NMPC/DFBC控制器 |
| `swarm_user_cmd` | `fsm_ctrl/swarm_user_cmd` | 终端交互界面，UDP命令发送 |
| `mavros` | `mavros/mavros` | MAVLink通信节点 |

### 关键话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/vrpn_client_node/jy0/pose` | PoseStamped | Mocap→Estimator | 动作捕捉原始位姿 |
| `/mavros/vision_pose/pose` | PoseStamped | Estimator→PX4 | 视觉位姿(供EKF融合) |
| `/mavros/local_position/pose` | PoseStamped | PX4→FSM | EKF融合后的位姿 |
| `/mavros/local_position/velocity_local` | TwistStamped | PX4→FSM | EKF融合后的速度 |
| `/mavros/setpoint_raw/attitude` | AttitudeTarget | FSM→PX4 | NMPC输出的bodyrate+thrust |
| `/mavros/state` | State | PX4→FSM | 飞控状态(mode, armed) |
| `/mavros/rc/in` | RCIn | PX4→FSM | 遥控器信号 |
| `/fsm_ctrl/ekf_ready` | Bool | Estimator→FSM | EKF融合收敛标志 |
| `/nmpc_state` | nmpc_state | FSM→外部 | NMPC状态调试信息 |

### 关键参数

| 参数 | 默认值 | 文件位置 | 说明 |
|------|--------|---------|------|
| `vision_source` | 0 | px4_estimator.launch:8 | 0=mocap, 1=lidar, 2=camera |
| `flag_mocap_frame` | 0 | px4_estimator.cpp:22 | 0=Z-up, 1=Y-up |
| `fcu_url` | `/dev/ttyACM0:921600` | single.launch:7 | 飞控串口 |
| `nmpc_hover_thrust` | 0.385 | single.launch:79 | 悬停油门(younger NMPC用) |
| `nmpc_Qpos{z}` | 50.0 | single.launch:98 | NMPC高度代价权重 |


现在出现了一个严重问题，需要你从多方面考虑检查。在悬停过程中，虽然飞起来的过程很正常，但是悬停总是掉高度，然后发生振荡。你可以读取~/bag_temp 
里的控制记录包裹，里面记录了详细的NMPC参数和飞行数据，分析他们。阅读这里的代码实现，分析一下。