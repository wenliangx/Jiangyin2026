/**
 * @file    dfbc
 * @brief   differential flatness based control
 * @author  FLAG Lab, BIT
 * @version 4.0 
 * @date    2024-07-10
 */

#include <fsm_ctrl/dfbc.hpp>



/**
 * @brief  DFBC class constructor
 * @param  _ctrl_rate: control rate
 * @param  _hover_thrust: hover thrust
 * @param  _kp_pos, _ki_pos, _kd_pos: position PID gain
 * @param  _kp_vel, _ki_vel, _kd_vel: velocity PID gain
 * @param  _kp_yaw, _kd_yaw: yaw PID gain
 * @return NULL
 */
DFBC::DFBC(int _ctrl_rate, double _hover_thrust, Eigen::Vector3d _kp_pos, Eigen::Vector3d _ki_pos, Eigen::Vector3d _kd_pos,
           Eigen::Vector3d _kp_vel, Eigen::Vector3d _ki_vel, Eigen::Vector3d _kd_vel, double _kp_yaw, double _kd_yaw)
{
    ctrl_interv = 1.0/_ctrl_rate;
    thrust_estor.Set_Estor(ctrl_interv, _hover_thrust);
    kp_pos = _kp_pos;
    ki_pos = _ki_pos;
    kd_pos = _kd_pos;
    kp_vel = _kp_vel;
    ki_vel = _ki_vel;
    kd_vel = _kd_vel;
    kp_yaw = _kp_yaw;
    kd_yaw = _kd_yaw;
    ierr_pos = Eigen::Vector3d::Zero();
    derr_pos = Eigen::Vector3d::Zero();
    ierr_vel = Eigen::Vector3d::Zero();
    derr_vel = Eigen::Vector3d::Zero();
    derr_yaw = 0.0;
}


/**
 * @brief  velocity + yaw rate control
 * @param  _ctrl_ref: control point reference
 * @param  _pos_fdb, _vel_fdb: position & velocity feedback
 * @param  _yaw_fdb: yaw feedback
 * @return geometry_msgs::TwistStamped command
 */
geometry_msgs::TwistStamped DFBC::VelCtrl(CtrlPt _ctrl_ref, Eigen::Vector3d _pos_fdb, Eigen::Vector3d _vel_fdb, double _yaw_fdb)
{
    /* position & yaw error */
    Eigen::Vector3d pos_err = _ctrl_ref.pos - _pos_fdb;
    double yaw_err = _ctrl_ref.euler(2) - _yaw_fdb;

    ierr_pos = ierr_pos + pos_err*ctrl_interv;
    ierr_pos = Eigen::Vector3d(Clamp_Single(ierr_pos(0), 5.0), Clamp_Single(ierr_pos(1), 5.0), Clamp_Single(ierr_pos(2), 5.0));
    derr_pos = pos_err - derr_pos;
    derr_yaw = yaw_err - derr_yaw;

    /* desired velocity & rate: position PID & yaw PD */
    ctrl_des.vel(0) = _ctrl_ref.vel(0) + kp_pos(0)*pos_err(0) + ki_pos(0)*ierr_pos(0) + kd_pos(0)*derr_pos(0);
    ctrl_des.vel(1) = _ctrl_ref.vel(1) + kp_pos(1)*pos_err(1) + ki_pos(1)*ierr_pos(1) + kd_pos(1)*derr_pos(1);
    ctrl_des.vel(2) = _ctrl_ref.vel(2) + kp_pos(2)*pos_err(2) + ki_pos(2)*ierr_pos(2) + kd_pos(2)*derr_pos(2);
    ctrl_des.rate(2) = kp_yaw*yaw_err + kd_yaw*derr_yaw;

    derr_pos = pos_err;
    derr_yaw = yaw_err;

    geometry_msgs::TwistStamped cmd = VelCmd();
    return cmd;
}


/**
 * @brief  differential flatness based control
 * @param  _ctrl_ref: control point reference
 * @param  _pos_fdb, _vel_fdb: position & velocity feedback
 * @return mavros_msgs::AttitudeTarget command
 */
mavros_msgs::AttitudeTarget DFBC::DFBCtrl(CtrlPt _ctrl_ref, Eigen::Vector3d _pos_fdb, Eigen::Vector3d _vel_fdb)
{   
    /*    desired velocity: position PI (D included in velocity loop)    */ 
	Eigen::Vector3d pos_err = _ctrl_ref.pos - _pos_fdb;
    ierr_pos = ierr_pos + pos_err*ctrl_interv;
    ierr_pos = Eigen::Vector3d(Clamp_Single(ierr_pos(0), 5.0), Clamp_Single(ierr_pos(1), 5.0), Clamp_Single(ierr_pos(2), 5.0));
    
    ctrl_des.vel(0) = _ctrl_ref.vel(0) + kp_pos(0)*pos_err(0) + ki_pos(0)*ierr_pos(0);
    ctrl_des.vel(1) = _ctrl_ref.vel(1) + kp_pos(1)*pos_err(1) + ki_pos(1)*ierr_pos(1);
    ctrl_des.vel(2) = _ctrl_ref.vel(2) + kp_pos(2)*pos_err(2) + ki_pos(2)*ierr_pos(2);

    /*    desired acceleration: velocity PID    */ 
    Eigen::Vector3d vel_err = ctrl_des.vel - _vel_fdb;
    ierr_vel = ierr_vel + vel_err*ctrl_interv;
    ierr_vel = Eigen::Vector3d(Clamp_Single(ierr_vel(0), 5.0), Clamp_Single(ierr_vel(1), 5.0), Clamp_Single(ierr_vel(2), 5.0));
    derr_vel = vel_err - derr_vel;
 	 	
    ctrl_des.acc(0) = _ctrl_ref.acc(0) + kp_vel(0)*vel_err(0) + ki_vel(0)*ierr_vel(0) + kd_vel(0)*derr_vel(0);
    ctrl_des.acc(1) = _ctrl_ref.acc(1) + kp_vel(1)*vel_err(1) + ki_vel(1)*ierr_vel(1) + kd_vel(1)*derr_vel(1);
    ctrl_des.acc(2) = _ctrl_ref.acc(2) + kp_vel(2)*vel_err(2) + ki_vel(2)*ierr_vel(2) + kd_vel(2)*derr_vel(2) + GRAVITY;

    derr_vel = vel_err;

    /* desired attitude */
    ctrl_des.euler(2) = _ctrl_ref.euler(2);
    ctrl_des.euler(1) = std::atan2(std::cos(ctrl_des.euler(2))*ctrl_des.acc(0) + std::sin(ctrl_des.euler(2))*ctrl_des.acc(1),
                                   std::abs(ctrl_des.acc(2)));
    ctrl_des.euler(0) = std::atan2((std::sin(ctrl_des.euler(2))*ctrl_des.acc(0) - std::cos(ctrl_des.euler(2))*ctrl_des.acc(1))*std::cos(ctrl_des.euler(1)),
                                    std::abs(ctrl_des.acc(2)));
    
    mavros_msgs::AttitudeTarget cmd = AttCmd(_ctrl_ref.euler(2));
    return cmd;
}


/**
 * @brief  generate attitude control command
 * @param  _yaw: yaw reference
 * @return mavros_msgs::AttitudeTarget
 */
mavros_msgs::AttitudeTarget DFBC::AttCmd(double _yaw)
{
    mavros_msgs::AttitudeTarget cmd;
    ctrl_des.thrust = thrust_estor.LSE(ctrl_des.acc.norm());
    ctrl_des.quat = EulerToQuat(ctrl_des.euler(0), ctrl_des.euler(1), _yaw);
	cmd.header.frame_id = "FCU";
	cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
	cmd.orientation.w = ctrl_des.quat.w();
    cmd.orientation.x = ctrl_des.quat.x();
	cmd.orientation.y = ctrl_des.quat.y();
	cmd.orientation.z = ctrl_des.quat.z();
	cmd.thrust = ctrl_des.thrust;
    return cmd;
}


/**
 * @brief  generate velocity control command
 * @param  NULL
 * @return geometry_msgs::TwistStamped command
 */
geometry_msgs::TwistStamped DFBC::VelCmd()
{
    geometry_msgs::TwistStamped cmd;
    cmd.header.frame_id = "world";
    cmd.twist.linear.x = ctrl_des.vel(0);
    cmd.twist.linear.y = ctrl_des.vel(1);
    cmd.twist.linear.z = ctrl_des.vel(2);
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = ctrl_des.rate(2);
    return cmd;
}
// airsim_ros::VelCmd DFBC::VelCmd()
// {
//     airsim_ros::VelCmd msg;
//     msg.twist.angular.z = ctrl_des.rate.yaw;         //yaw角速度
//     msg.twist.linear.x = ctrl_des.vel(0);            //x线速度
//     msg.twist.linear.y = ctrl_des.vel(1);;           //y线速度
//     msg.twist.linear.z = ctrl_des.vel(2);;           //z线速度
//     return msg;
// }