Header header
super_msgs/PositionCommand[] cmds   # MPC 需要的一串轨迹点
uint32 mpc_horizon                  # MPC 的预测步数（规划了多少个未来点）
uint8 command_flag                  # 命令类型标志
uint8 NORMAL_COMMAND = 1            # 正常轨迹命令
uint8 BLOCK_COMMAND = 137           # 阻塞/急停命令
