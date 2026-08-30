# 仓库清理边界

清理以 `origin/master` 的比赛通过提交 `d23205d` 为行为基线。原则是先删除可由
构建重新生成或只用于上游展示的内容，不在没有明确确认时删除飞行、仿真或部署
入口。

## 已清理

- `logs/`：154 个误提交的 Catkin 构建日志；目录已加入 `.gitignore`。
- `src/SUPER/misc/` 中约 75 MB 的论文 PDF 和演示 GIF；README 已改为外链。
- 根 README 中已失效的构建日志链接，并补充当前 SML 主流程入口。

这些删除均可从 Git 历史恢复，不改变编译目标、ROS launch 或运行参数。

## 已迁移并删除

- 旧的单机控制节点、对应 launch 和悬停指南已删除。
- Docker、tmux 和仿真入口已统一迁移到 `flight_fsm.launch`。
- AI 助手生成的仓库内配置和过程文件已删除并加入忽略规则。
- 未使用的 tmux 启动脚本、ego-planner-v2、gz_external_pose 和 px4_plugs 已删除；
  仿真与部署脚本已同步清理对应入口。

## 已审计但保留

以下内容看起来属于旧流程，但仍存在引用链，后续应单独确认后再删除：

- `swarm_user_cmd`、`swarm.launch`：仍在使用的多机控制入口。
- `src/plane_Det/`：带 `CATKIN_IGNORE`，当前没有活动启动入口。
- `sim_config/`：仍承载当前仿真入口。
- SUPER 的 PCD 地图和 `yunque-M.dae`：体积较大，但仍被
  `perfect_drone_sim/config/*.yaml` 使用。
