# 悬停控制分析报告 — 2026-08-13-16-22-43.bag

分析脚本：`extract_bag.py`, `analyze_hover.py`, `analyze2.py`, `analyze3.py`, `analyze4.py`, `check_prev.py`
图表：`hover_analysis.png`

## 飞行概况

- NMPC 激活窗口 t = 4.8–18.1 s（共 13.4 s），之后降落。
- 状态机为 `MissionMachine` → `NmpcHover` → `TickLowerHover`，目标点固定为 **(0, 0, 0.4)**，horizon 9 点全为该常值。
- NMPC 求解频率 **50 Hz 正常**（中位间隔 19.8 ms，最大 25.6 ms）——控制环本身没掉速。

## 实测性能

| 指标 | 数值 | 评价 |
|---|---|---|
| z 稳态误差 | **-0.087 m**（悬停在 0.31 而不是 0.40） | 系统性偏低，两次飞行一致（16-20-03 包也是 -0.082 m） |
| z 摆动 | std 0.019 m | 好 |
| x 稳态误差 | 均值 +0.079 m，游走 ±0.2 m，p2p 0.21 m | 差，恢复极慢（时间常数 ~4 s） |
| y 稳态误差 | 均值 0.001 m，std 0.031 m | 尚可 |
| 稳态推力指令 | **均值 0.456**（参数 nmpc_hover_thrust = 0.430） | 参数比真实悬停油门低 ~6% |
| 俯仰角速度指令 wy | 存在 **~1.6 Hz 抖动**（幅值 0.16 rad/s），与 x 误差几乎零相关 | 无效高频抖动 |
| 体角速度→速度响应 | wy→vx 在 80 ms 滞后处相关 -0.69 | PX4 内环跟踪正常，问题不在飞控 |

## 诊断结论

### 1. 高度偏低 9 cm 的根因：nmpc_hover_thrust 比真实悬停油门低 6%

- 参数 0.430，实测稳态推力指令均值 **0.456**（两个包一致，16-20-03 包为 0.447）。
- 推力气动换算 `ThrEst` 是**无效的**：`LSE()` 用"指令加速度 vs 指令推力"做递推（`acc_z_command = thr_est.LSE(cmd_control_acc_z.front())`，见 NMPC_Controller.cpp:275），两者永远按当前换算系数互相生成，是数学上的不动点——**永远学不到真实质量/油门特性**。
- 因此 NMPC 必须长期输出 acc_z ≈ 10.2–10.4 m/s²（vs g=9.80）才能不掉高；`RtotalF=0.15` 惩罚输入偏离 g，优化器就在"输入多出力"和"高度误差"之间做权衡，最终平衡在 **z 偏低 8–9 cm** 的位置。
- **修法**：`nmpc_hover_thrust` 0.430 → **0.455**（实测值；地面效应影响，建议每次换场地/换电池后重新量一次稳态推力）。

### 2. 水平游走 ±0.2 m、恢复慢的根因：预测时域太短 + Qvel 惩罚过大

- 当前 `NLP_predict_step=8 × 0.05 s = 0.4 s` 的时域。修正 0.2 m 的误差需要先倾转→加速→再倾转回来，0.4 s 内做完不划算（还要付 Qvel=4 的速度惩罚），优化器就选择"慢慢来"——数据里 x 误差以 **~4 s 时间常数**衰减，wy 指令与 x 误差相关系数仅 ~0.09。
- y 轴好一些是因为 Qposy=55 > Qposx=45 且初始误差小，不是结构性问题。
- **修法**（按优先级）：
  1. 时域加长：`NLP_predict_step` 8 → **16**（onestep 0.05，共 0.8 s），或 10 × 0.08 s。注意这是在 `single_offboard_sml.cpp:385-389` 构造函数里写死的，需要改代码（顺便把 step/onestep 提成参数）。
  2. `nmpc_Qvelx/y` 4.0 → **1.5~2.0**：不要惩罚"向目标运动"的速度。
  3. `nmpc_Qposx` 45 → **55**（与 y 一致）。
  4. 如求解时间吃紧（当前 20 ms，时域加长会变慢），`ipopt.print_level` 5 → 0 可省掉大量打印开销。

### 3. 俯仰指令 1.6 Hz 抖动的根因：IPOPT 过早终止 + 无输入平滑

- `ipopt.acceptable_tol = 1e-4`（NMPC_Controller.cpp:141）让求解器"差不多就停"，带热启动的相邻两次解会有符号级抖动；位置反馈噪声（视觉定位 ~2-3 cm）被放大成 wy 的高频抖动。位置是二阶积分环节，1.6 Hz 的抖动对位置几乎无影响，但消耗舵效、看着也糟。
- **修法**：
  1. `acceptable_tol` 1e-4 → **1e-6**（更精确的解；配合 print_level=0 控制耗时）；
  2. `nmpc_Rwx/y` 0.45 → **0.6~0.8**（提高角速度代价，平滑指令）；
  3. 兜底方案：发布前对 body_rate 做一阶低通（EMA 权重 ~0.6），一行代码、效果立竿见影。

### 4. 代码问题（顺手修掉）

- [single_offboard_sml_actions.hpp:85-88](src/fsm_ctrl/include/fsm_ctrl/single_offboard_sml_actions.hpp#L85-L88)：`TickLowerHover` 把目标硬编码成 **(0,0,0.4)**——悬停会飞向地图原点而不是保持当前位置（工作区改动把原来的"保持当前 x/y"改掉了）。悬停测试请改回保持当前 x/y。
- [single_offboard_sml.cpp:426](src/fsm_ctrl/src/single_offboard_sml.cpp#L426) 与 actions 里的 `std::cout << "hello"`：**50 Hz 的 stdout 输出**，会造成控制环抖动（已观测到最大求解间隔 25.6 ms），务必删掉。
- 次要观察：稳态 wz 指令恒为 -0.037 rad/s 但 yaw 几乎不动。`AttitudeTarget.body_rate` 是 **FRD 系**（yaw 速率正方向 = 顺时针），而 NMPC 按 ENU/FLU 建模——**wz 符号可能反了**，建议确认一下 yaw 速率方向。

### 5. 鲁棒性建议（可选，比赛前建议做）

- NMPC 无积分环节，任何持续偏置（电池压降、地面效应、视觉零漂、桨不平衡）都会变成稳态误差。建议：
  - z 轴参考做慢速积分修正（参考 = 0.4 + k_I ∫e_z dt，限幅 ±0.1 m）——最便宜有效；
  - 或者修好 ThrEst：把真实测量的 z 加速度（如 `/mavros/imu/data`）喂进去，而不是喂指令加速度。

## 参数调整汇总表

| 位置 | 参数/代码 | 现在 | 建议 | 目的 |
|---|---|---|---|---|
| launch 132 | nmpc_hover_thrust | 0.430 | **0.455** | 消除 -9 cm 高度偏差 |
| sml.cpp:387 | NLP_predict_step | 8 | **16**（时域 0.8 s） | 加快水平纠偏 |
| sml.cpp:387 | NLP_onestep_time | 0.05 | 0.05（保持） | 求解时间可控 |
| launch 124-126 | nmpc_Qvelx/y | 4.0 | **1.5~2.0** | 不再压制纠偏速度 |
| launch 120 | nmpc_Qposx | 45 | **55** | 与 y 轴一致 |
| launch 132-133 | nmpc_Rwx/y | 0.45 | **0.6~0.8** | 压制 1.6 Hz 抖动 |
| NMPC_Controller.cpp:141 | ipopt.acceptable_tol | 1e-4 | **1e-6** | 精确解，减少抖动 |
| NMPC_Controller.cpp:139 | ipopt.print_level | 5 | 0 | 省打印时间 |
| actions.hpp:85-88 | TickLowerHover 目标 | (0,0,0.4) 硬编码 | 保持当前 x/y | 悬停语义正确 |
| actions.hpp:87 / sml.cpp:426 | std::cout 50Hz 打印 | 有 | 删除 | 消除环抖动 |

改动后复飞验证：z 应贴住 0.40 m（±3 cm）、x/y 游走应收敛到 ±5 cm 内且纠偏明显变快、wy 抖动幅值下降。
