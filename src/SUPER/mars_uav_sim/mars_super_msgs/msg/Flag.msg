Header header
geometry_msgs/Point position       # 当前目标位置（xyz）
super_msgs/PositionCommand[] cmd   
int16 id                           # 航点编号（0,1,2,...）
int16 now_id                       # 当前已经飞到的航点编号
int16 mode                         # 速度模式（0=max_vel0, 1=max_vel1, 2=max_vel2）
int16 is_map                       # 是否启用避障（1=启用, 0=关闭）
float32 yaw                        # 终端 yaw（rad）；NaN 表示不固定终端 yaw
float32 desired_speed              # 终端速度大小（m/s）；小于等于 0 时终端速度为 0
int16 total_waypoint               # 航点总数
int16 touch_goal                   # 是否到达目标
int16 touch_obstacle               # 是否碰到障碍物
