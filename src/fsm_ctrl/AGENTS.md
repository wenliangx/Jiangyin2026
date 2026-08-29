# fsm_ctrl — AGENTS.md

**FSM state machine + NMPC guidance controller for Jiangyin2026 UAV. Pipeline: RA-LIO (odom) → px4_estimator (EKF2 fusion) → single_offboard (guidance) → MAVROS → PX4 (actuation).**

## BINARIES

| Binary | Source | Role |
|--------|--------|------|
| `single_offboard_fsm` | `src/single_offboard_fsm.cpp` (2829L) | Legacy FSM + NMPC controller (UDP cmd dispatcher) |
| `flight_fsm` | `src/flight_fsm_node.cpp` | Boost.SML segmented mission FSM (9 states, 50Hz) |
| `px4_estimator` | `src/px4_estimator.cpp` | MoCap/RA-LIO pose → `/mavros/odometry/out` for PX4 EKF2 fusion (8 subs) |
| `swarm_user_cmd` | `src/swarm_user_cmd.cpp` | Multi-drone UDP command parser |

## SOURCE MAP

```
src/
├── single_offboard_fsm.cpp    # Legacy FSM — UDP recv, mode switching, Set_TargetPosition/Mavros
├── flight_fsm_node.cpp       # ROS setpoint, NMPC, mission, landing, camera adapters
├── px4_estimator.cpp          # 8 subscribers (mocap, odom, IMU, RC); publishes `/mavros/odometry/out`
├── NMPC_controller.cpp        # CasADi NMPC solver wrapper (NMPC_Ctrller / NMPC_Ctrller_simple)
├── att_nmpc.cpp               # Attitude-level NMPC (Q/R weights, thrust estimator)
├── dfbc.cpp                   # Dynamic-force-based control fallback
├── ctrl_math.cpp              # Quat↔Euler, ThrEst (LSE thrust estimation with rho=0.998 forgetting factor)
├── traj_gen.cpp               # Internal trajectory generators (circle, parametric)
└── swarm_user_cmd.cpp         # Multi-drone UDP command parser
include/fsm_ctrl/
├── fsm/                       # ROS-independent StateMachine + command edge dispatcher
├── flight_fsm.hpp            # Flight-domain convenience include
├── flight_fsm/
│   ├── types.hpp / ports.hpp / context.hpp
│   ├── states.hpp / events.hpp
│   ├── actions.hpp / landing_planner.hpp
│   └── machine.hpp / command_dispatcher.hpp / command_metadata.hpp
├── single_offboard_fsm.hpp    # Legacy FSM class
├── nmpc_ctrl.hpp              # NMPC_Ctrller interface (CasADi optimal_solution)
├── att_nmpc.hpp               # Attitude NMPC (MatQ, MatR, step_horizon)
├── dfbc.hpp                   # DFBC interface
├── ctrl_math.hpp              # CtrlPt, ThrEst, quaternions, GRAVITY=9.8015
├── NMPC_test.hpp / NMPC_Controller.hpp  # Solver library headers
├── px4_estimator.hpp          # Global state for mocap/odom callbacks
└── traj_gen.hpp               # TrajPoint, TrajCircle
third_party/boost_sml/         # Boost.SML header-only (~2800 LOC)
msg/
├── nmpc_state.msg             # Full NMPC debug state (pos_ref[], vel_ref[], pos_fdb, vel_fdb, target)
└── younger_debug.msg          # Legacy debugging output
launch/
├── flight_fsm.launch / flight_fsm_uav2.launch # Competition profiles
├── single.launch                       # Legacy single_offboard_fsm config
├── swarm.launch                        # Multi-drone launch
└── px4_estimator.launch                # px4_estimator node config
test/
├── state_machine_framework_test.cpp # 2 focused tests for the reusable framework
├── flight_fsm_test.cpp     # 38 competition-machine GTests
└── flight_fsm_smoke.test.in # configured rostest with sealed test waypoints
```

## CONVENTIONS

- **C++14** for both legacy and SML targets. Links `/usr/local/lib/libcasadi.so.3.7`
- **Reusable framework**: `fsm::StateMachine<Definition, TickEvent>` owns the Boost.SML runtime; `fsm::DistinctCommandDispatcher` performs protocol-independent command-edge routing. Both are header-only and ROS/CasADi independent.
- **SML FSM**: active `SegmentedMissionMachine` has 9 states and a 50Hz tick loop; UDP commands are dispatched as `OnCommand*` events. `MissionMachine` remains for focused tests/alternate composition.
- **Vision camera control**: every SML Tick publishes a complete latched `uav_vision_msgs/VisionControl` snapshot. In the active segmented mission, `SuperSegment1` enables only the front camera, `SuperSegment2` enables only the rear camera through the legacy `down_camera_enabled` field, and every other state disables both cameras.
- **Legacy FSM**: `Set_TargetPosition` for cmd1-4, `AttitudeTarget` for cmd5-8
- **Idle cameras**: both mission machines publish front-camera and down-camera enabled while in Idle; Idle still produces no flight control output and never arms.
- **Waypoint preflight**: edit `config/mission_super_waypoints.yaml` and the independent `config/waypoint_validation_rules.yaml`, then seal them with `validate_super_waypoints.py`. Flight launch files load only `config/validated/mission_super_waypoints.validated.yaml`. Startup verifies read-only permissions, both SHA-256 hashes, and all rules; failure aborts the node and never falls back to built-in waypoints.
- **Landing completion**: Landing entry uses `ResetLanding`, matching the active `TickLanding/prepareLanding` path; it does not initialize the unused closed-loop planner. Neither landing Tick path requests OFFBOARD or arm. Completion sets a process-lifetime permanent safety lock, latches low-thrust output, and immediately requests a normal disarm through `/mavros/cmd/arming`, retrying at `service_retry_seconds` while telemetry remains armed. Once telemetry reports disarmed, thrust publication stops. The permanent lock rejects every later state command and blocks all automatic arm requests, so only restarting the process can clear it. Landing vision observations are currently monitor-only and do not participate in the control loop.
- **Testing**: 40 GTests (2 framework + 38 competition machine/planner), 3-case rostest smoke, and 5 waypoint-validator tests. The smoke test generates and verifies an isolated read-only waypoint seal in the build tree. Hand-rolled fakes for 7 interfaces, including `FakeCameraControl`; no gmock.
- **NMPC_test.cpp** is production controller code (FLAG_NMPC library), NOT a test file despite its name
- **NMPC weights**: ROS params (`nmpc_Qpos*`, `nmpc_Rwx`). Horizon: 10pt @ 0.05s, 8-step MPC
- **Thrust estimation**: `ThrEst::LSE()` RLS with `rho=0.998` — DO NOT CHANGE
- **Boost.SML**: header-only in `third_party/` (not system dep)

## ANTI-PATTERNS

- **NEVER** mix pos+velocity+accel + yaw+yaw_rate in single `Set_TargetPosition` call (line 249 of single_offboard_fsm.cpp) — corrupts target
- **NEVER** use `aim_pos` / `aim_vel` for attitude control — position/velocity fields only
- **NEVER** change `rho=0.998` in ctrl_math.hpp — breaks thrust estimation LSE fit
- FIXME left in constructor initializer list (`use_external_yaw_` member)
- NMPC solve time >10ms means control loop budget is tight — monitor `solve()` timing
- `setpoint_raw/attitude` with `IGNORE_ATTITUDE` = body-rate + thrust mode (no attitude command)
- `position_target` mask bits must be set correctly — wrong mask = PX4 ignores commands silently
