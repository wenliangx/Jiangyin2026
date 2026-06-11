/**
 * @file    ctrl_math
 * @brief   math functions used for control
 * @author  FLAG Lab, BIT
 * @version 1.1 add thrust estimation
 * @date    2024-07-09
 * @note    Based on actual flight conditions, roll and pitch are within -PI/2 ~ PI/2. 
 *          Due to the range limit of atan2(), conversions to yaw only yield -PI ~ PI.
 *          If yaw is within -PI ~ PI, quaternion will have the limit of w > 0 with a singular point.
 */

#include <fsm_ctrl/ctrl_math.hpp>



/**
 * @brief  convert quaternion to euler angle
 * @param  _w, _x, _y, _z: quaternion
 * @return euler angle
 */
Eigen::Vector3d QuatToEuler(double _w, double _x, double _y, double _z)
{
    Eigen::Vector3d euler;

    double sinr = 2.0*(_w*_x + _y*_z);
    double cosr = 1.0 - 2.0*(_x*_x + _y*_y);
    euler[0] = std::atan2(sinr, cosr);

    double sinp = 2.0*(_w*_y - _z*_x);
    if(std::abs(sinp) >= 1) {euler[1] = std::copysign(M_PI/2, sinp);}
    else                    {euler[1] = std::asin(sinp);}

    double siny = 2.0*(_w*_z + _x*_y);
    double cosy = 1.0 - 2.0*(_y*_y + _z*_z);
    euler[2] = std::atan2(siny, cosy);
    
    return euler;
}

Eigen::Vector3d QuatToEuler(Eigen::Quaterniond _quat)
{
    Eigen::Vector3d euler = QuatToEuler(_quat.w(), _quat.x(), _quat.y(), _quat.z());
    return euler;
}


/**
 * @brief  convert euler angle to quaternion 
 * @param  _roll, _pitch, _yaw: euler angle
 * @return quaternion
 */
Eigen::Quaterniond EulerToQuat(double _roll, double _pitch, double _yaw)
{
    Eigen::Quaterniond quat;
    double cr = std::cos(_roll*0.5);
    double sr = std::sin(_roll*0.5);
    double cp = std::cos(_pitch*0.5);
    double sp = std::sin(_pitch*0.5);
    double cy = std::cos(_yaw*0.5);
    double sy = std::sin(_yaw*0.5);
    quat.w() = cr*cp*cy + sr*sp*sy;
    quat.x() = sr*cp*cy - cr*sp*sy;
    quat.y() = cr*sp*cy + sr*cp*sy;
    quat.z() = cr*cp*sy - sr*sp*cy;
    return quat;
}

Eigen::Quaterniond EulerToQuat(Eigen::Vector3d _euler)
{
    Eigen::Quaterniond quat = EulerToQuat(_euler[0], _euler[1], _euler[2]);
    return quat;
}


/**
 * @brief  convert quaternion to rotation matrix
 * @param  _w, _x, _y, _z: quaternion
 * @return rotation matrix
 */
Eigen::Matrix3d QuatToMat(double _w, double _x, double _y, double _z)
{
    Eigen::Matrix3d mat;
    mat << _w*_w + _x*_x - _y*_y - _z*_z, 2*(_x*_y - _w*_z), 2*(_x*_z + _w*_y),
           2*(_x*_y + _w*_z), _w*_w - _x*_x + _y*_y - _z*_z, 2*(_y*_z - _w*_x),
           2*(_x*_z - _w*_y), 2*(_y*_z + _w*_x), _w*_w - _x*_x - _y*_y + _z*_z;
    return mat;
}

Eigen::Matrix3d QuatToMat(Eigen::Quaterniond _quat)
{
    Eigen::Matrix3d mat;
    mat = QuatToMat(_quat.w(), _quat.x(), _quat.y(), _quat.z());
    return mat;
}


/**
 * @brief  convert euler angle to rotation matrix
 * @param  _roll, _pitch, _yaw: euler angle
 * @return rotation matrix
 */
Eigen::Matrix3d EulerToMat(double _roll, double _pitch, double _yaw)
{
    Eigen::Matrix3d mat_r, mat_p, mat_y;
    mat_r << 1.0,    0.0,         0.0,
             0.0, cos(_roll), -sin(_roll),
             0.0, sin(_roll),  cos(_roll);
    mat_p << cos(_pitch), 0.0, sin(_pitch),
                0.0,      1.0,    0.0,
            -sin(_pitch), 0.0, cos(_pitch);
    mat_y << cos(_yaw), -sin(_yaw), 0.0,
             sin(_yaw),  cos(_yaw), 0.0,
                0.0,        0.0,    1.0;
    Eigen::Matrix3d mat = mat_y*mat_p*mat_r;
    return mat;
}

Eigen::Matrix3d EulerToMat(Eigen::Vector3d _euler)
{
    Eigen::Matrix3d mat = EulerToMat(_euler[0], _euler[1], _euler[2]);
    return mat;
}


/**
 * @brief  convert rotation matrix to euler angle
 * @param  _mat: rotation matrix
 * @return euler angle
 */
Eigen::Vector3d MatToEuler(Eigen::Matrix3d _mat)
{
    Eigen::Vector3d euler;
    euler[0] = atan2(_mat(2,1), _mat(2,2));
    euler[1] = asin(-_mat(2,0));
    euler[2] = atan2(_mat(1,0), _mat(0,0));
    return euler;
}


/**
 * @brief  convert rotation matrix to quaternion
 * @param  _mat: rotation matrix
 * @return quaternion
 */
Eigen::Quaterniond MatToQuat(Eigen::Matrix3d _mat)
{
    Eigen::Quaterniond quat;
    quat.w() = sqrt(1.0 + _mat(0,0) + _mat(1,1) + _mat(2,2))*0.5;
    quat.x() = ( _mat(2,1) - _mat(1,2) )/(4*quat.w());
    quat.y() = ( _mat(0,2) - _mat(2,0) )/(4*quat.w());
    quat.z() = ( _mat(1,0) - _mat(0,1) )/(4*quat.w());
    return quat;
}


/**
 * @brief  clamp with single limit 
 * @param  _data: raw value
 * @param  _max: limit
 * @return clamped value
 */
double Clamp_Single(double _data, double _max)
{
    double data = Clamp_Double(_data, -_max, _max);
    return data;
}


/**
 * @brief  clamp with double limit 
 * @param  _data: raw value
 * @param  _min, _max: lower & upper limit
 * @return clamped value
 */
double Clamp_Double(double _data, double _min, double _max)
{
    if(_data < _min) {return _min;}
    if(_data > _max) {return _max;}
    return _data;
}


/**
 * @brief  sign function
 * @param  _data: raw value
 * @return sign of the value
 */
double Sign(double _data)
{
    if(_data > 0.0) {return 1.0;}
    if(_data < 0.0) {return -1.0;}
    return 0.0;
}


/**
 * @brief  thrust estimator setup 
 * @param  _ctrl_interv: control interval
 * @param  _hover_thrust: hover thrust
 * @return NULL
 */
void ThrEst::Set_Estor(int _ctrl_rate, double _hover_thr)
{
    ctrl_dt = 1.0/_ctrl_rate;
    hover_thr = _hover_thr;
    thr_to_acc = GRAVITY/_hover_thr;
}


/**
 * @brief  recursive least squares thrust estimation with forgetting factor
 * @param  _acc: acceleration
 * @return thrust
 */
double ThrEst::LSE(double _acc)
{
    ros::Time now = ros::Time::now();
    if(thr_stamped.size() == 0)
    {
        thr_stamped.push(std::pair<ros::Time, double>(now, _acc/thr_to_acc));
        ROS_WARN("No data for thrust estimation! Slope unchanged!");
        return _acc/thr_to_acc;
    }
    else
    {
        std::pair<ros::Time, double> time_thr_pair = thr_stamped.front();
        double time_pass = (now - time_thr_pair.first).toSec();
        // std::cout << "time_pass: " << time_pass << std::endl;
        if(time_pass > 1.5*ctrl_dt) {ROS_WARN("Control frequency lower than ROS rate!");}
        
        /* successful estimation */
        double thr = time_thr_pair.second;
        thr_stamped.pop();

        /* model: acc = thrust_to_acc * thrust */
        double gamma = 1.0/(rho + thr*p_est*thr);
        double k_est = gamma*p_est*thr;
        thr_to_acc = thr_to_acc + k_est*(_acc - thr*thr_to_acc);
        p_est = (1.0 - k_est*thr)*p_est/rho;
        thr = _acc/thr_to_acc;

        thr_stamped.push(std::pair<ros::Time, double>(now, thr));
        return thr;
    }
}

/**
 * @brief  command to only arm
 * @param  NULL
 * @return mavros_msgs::AttitudeTarget command
 */
mavros_msgs::AttitudeTarget ArmCmd()
{
    mavros_msgs::AttitudeTarget cmd;
	cmd.header.frame_id = "FCU";
	cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
	cmd.orientation.w = 1.0;
    cmd.orientation.x = 0.0;
	cmd.orientation.y = 0.0;
	cmd.orientation.z = 0.0;
	cmd.thrust = 0.05;
    return cmd;
}


/**
 * @brief  approach yaw reference smoothly
 * @param  _yaw: current yaw command
 * @param  _yaw_ref: yaw reference
 * @return updated yaw command
 */
double YawSmooth(double &_yaw, double _yaw_ref)
{
    if(_yaw_ref - _yaw > M_PI/200) {_yaw += M_PI/200;}
    else if(_yaw_ref - _yaw < -M_PI/200) {_yaw -= M_PI/200;}
    else {_yaw = _yaw_ref;}
    return _yaw;
}