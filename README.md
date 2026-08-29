# Jiangyin2026

2026 无人机比赛代码。当前比赛主流程为：

`RA-LIO -> px4_estimator -> flight_fsm -> PX4`

控制器使用 Boost.SML 分段任务状态机与 NMPC，视觉模块提供目标分类和降落观测。
比赛航点在启动前必须经过校验并封存，详见
[`docs/waypoint_validation.md`](docs/waypoint_validation.md)。

常用入口：

- 实机控制：`roslaunch fsm_ctrl flight_fsm.launch`
- 备用参数组：`roslaunch fsm_ctrl flight_fsm_uav2.launch`
- 状态机测试：`catkin_make run_tests_fsm_ctrl`
- 可复用状态机框架：[`docs/state_machine_framework.md`](docs/state_machine_framework.md)

仓库清理边界与待确认项目见
[`docs/repository_cleanup.md`](docs/repository_cleanup.md)。
