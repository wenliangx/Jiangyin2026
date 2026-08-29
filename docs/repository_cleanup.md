# 仓库清理边界

清理以 `origin/master` 的比赛通过提交 `d23205d` 为行为基线。原则是先删除可由
构建重新生成或只用于上游展示的内容，不在没有明确确认时删除飞行、仿真或部署
入口。

## 已清理

- `logs/`：154 个误提交的 Catkin 构建日志；目录已加入 `.gitignore`。
- `src/SUPER/misc/` 中约 75 MB 的论文 PDF 和演示 GIF；README 已改为外链。
- 根 README 中已失效的构建日志链接，并补充当前 SML 主流程入口。

这些删除均可从 Git 历史恢复，不改变编译目标、ROS launch 或运行参数。

## 已审计但保留

以下内容看起来属于旧流程，但仍存在引用链，后续应单独确认后再删除：

- `single_offboard_fsm`、`swarm_user_cmd`、`single.launch`、`swarm.launch`：
  legacy 控制路径；`sim_config/` 和两个 Docker 启动脚本仍引用它们。
- `src/plane_Det/`：带 `CATKIN_IGNORE`，但 `tmux-real.sh` 仍引用它。
- `tmux-real.sh`、`tmux-sim.sh`、`sim_config/`、`NMPC_HOVER_GUIDE.md`：
  与 legacy FSM 成组的旧仿真/调参入口。
- SUPER 的 PCD 地图和 `yunque-M.dae`：体积较大，但仍被
  `perfect_drone_sim/config/*.yaml` 使用。
- ego-planner-v2 中带 `CATKIN_IGNORE` 的上游工具包：默认不编译，但仍有相互
  引用，适合在单独的 vendor 裁剪变更中处理。

建议把 legacy 控制路径作为一个原子变更删除：同时迁移或删除所有 Docker、tmux、
仿真和文档引用，并在容器内完成全量 Catkin 构建与 SML smoke test。不要只删除
可执行文件而保留失效入口。
