# NMPC 实际生效参数的只读镜像，由 single_offboard_fsm 启动时发布，
# 仅用于录包/回放观察，不参与任何控制回路。
float64 q_pos_x
float64 q_pos_y
float64 q_pos_z
float64 q_vel_x
float64 q_vel_y
float64 q_vel_z
float64 q_quat_x
float64 q_quat_y
float64 q_quat_z
float64 r_w_x
float64 r_w_y
float64 r_w_z
float64 r_acc_z        # nmpc_RtotalF，推力项 R 权重
float64 hover_thrust
float64 ctrl_t         # 控制周期 kInterval
float64 acc_z_limit_low
float64 acc_z_limit_high
float64 w_limit_low
float64 w_limit_high
int32 nlp_predict_step
float64 nlp_onestep_time
int32 nlp_state_num
int32 nlp_input_num
