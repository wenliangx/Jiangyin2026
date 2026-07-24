Header header
geometry_msgs/Point position       # 当前目标位置（xyz）
float32 yaw                        # 最终偏航角（rad）；NaN 表示不固定最终 yaw
float32 desired_speed              # 终端速度（m/s）；小于或等于 0 时终端速度为 0
super_msgs/PositionCommand[] cmd   
int16 id                           # 航点编号（0,1,2,...）
int16 now_id                       # 当前已经飞到的航点编号
int16 mode                         # 飞行模式（1=正常速度, 2=慢速, 3=降落）
int16 is_map                       # 是否启用避障（1=启用, 0=关闭）
int16 total_waypoint               # 航点总数
int16 touch_goal                   # 是否到达目标
int16 touch_obstacle               # 是否碰到障碍物
