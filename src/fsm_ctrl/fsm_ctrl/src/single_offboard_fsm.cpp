/**
 * @file    single_offboard_fsm
 * @brief   finite state machine for single drone
 * @author  FLAG Lab, BIT
 * @version 4.0
 * @date    2024-06-03
 */

#include <fsm_ctrl/single_offboard_fsm.hpp>
#include <fsm_ctrl/traj_gen.hpp>

using namespace std;

#define RATE 50.0
#define INTERV 1.0/RATE
#define HEIGHT 1.0 



/*--------------------------- FSM & Control ---------------------------*/

static bool apland = false;

static bool is_udp_enable = true;         // 1: enable receiving udp command  
static bool is_ready_fly = false;         // 0: wait for EKF pose fusion converging
static int cmd = 0;
  
static bool is_quat_init = false;         // 0: wait to initialize quaternion fusion
static bool is_need_rot = false;          // 1: initial qw < 0, need to rotate 360 (deg) on yaw axis

static bool is_land = false;            // 0: wait to land near ground 
static bool is_takeoff = false;         // 0: wait for RC confirming

static bool is_get_planner_msgs = false;

static int takeoff_channel = 0;
static int emergency_channel = 0;

mavros_msgs::State current_state;
static double voltage;

static Eigen::Vector3d pos_fcu;    
static Eigen::Vector3d vel_fcu;     
static Eigen::Quaterniond quat_fcu;
static Eigen::Vector3d euler_fcu;

static Eigen::Vector3d acc_imu;     
static Eigen::Quaterniond quat_imu;    
static Eigen::Vector3d euler_imu;
static Eigen::Vector3d rate_imu;

static Eigen::Vector3d pos_odom;
static Eigen::Vector3d vel_odom;
static Eigen::Quaterniond quat_odom;
static Eigen::Vector3d euler_odom;

geometry_msgs::PoseStamped aim_pos;          // position xyz + yaw, DO NOT use for attitude control
geometry_msgs::TwistStamped aim_vel;         // velocity xyz + yaw rate, DO NOT use for attitude control   
mavros_msgs::PositionTarget target_pos;         // position & velocity & acceleration + yaw & yaw rate
mavros_msgs::AttitudeTarget target_att;         // attitude & body rate + thrust
geometry_msgs::PoseStamped april_pos; 

vector<CtrlPt> traj_nmpc;

static bool ap_land = false;
static bool dynamic_ring = false;
static bool set_dynamic_ring = false;
static int task1 =0;

/*----------LXK----------*/
static int task; 
//1密林，2集装箱，3迷宫，4静环，5动环，6动平台 
static int task_print_count = 0;
static int state2task[] = {1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5};
static int ego_end_lock = 0;
std_msgs::Int64 perc_mode_msg;
static int perc_mode;
static int perc_mode_print_count = 0;
static Eigen::Quaterniond quat_yaw;
static double yaw_now;
static int yaw_print_count = 0;

static vector<TypePoint> Ego_traj;
static int Ego_traj_count = 0;                     // 当前飞往点的标号
static int Ego_traj_size = 46;
static bool need_GeneTraj = true;                 // 是否需要修改轨迹

//存放历史一段时间内的位姿
static vector<vector<double>> fsm_pos;                  //存放历史一段时间内的位姿
static int fsm_pos_num = 200;                           //存放历史位姿的容量
ros::Time ini_time;                                     //位姿初始时间
static bool is_ini_time = false;                        //是否初始化位姿时间

//tunnel
static Eigen::Vector3d tunnel_pose1 = {16.60, -0.44, 1.2};
static vector<Eigen::Vector3d> tunnel_filter1;
static Eigen::Vector3d tunnel_pose2 = {22.33, 1.147, 1.2};
static vector<Eigen::Vector3d> tunnel_filter2;
static Eigen::Vector3d tunnel_pose3 = {19.43, 0.1, 1.95};
static vector<Eigen::Vector3d> tunnel_filter3;
static int tunnel_filter_size = 9;

//maze
static Eigen::Vector3d maze_pose1 = {19.11, 11.59, 1.8};
static vector<Eigen::Vector3d> maze_filter1;
static Eigen::Vector3d maze_pose2 = {21.78, 14.16, 2.1};
static vector<Eigen::Vector3d> maze_filter2;
static Eigen::Vector3d maze_pose3 = {18.77, 16.40, 1.8};
static vector<Eigen::Vector3d> maze_filter3;
static int maze_filter_size = 5;

//ring1
static Eigen::Vector3d ring1_pose = {3.3, -0.2, 1.7};
static vector<Eigen::Vector3d> ring1_filter;
static int ring1_filter_size = 5;
//ring3
static Eigen::Vector3d ring3_pose = {7.2, -0.8, 1.0};
static vector<Eigen::Vector3d> ring3_filter;
static int ring3_filter_size = 5;
//ring2
static Eigen::Vector3d ring2_pose = {2.0, 0.0, 1.0};   
static vector<Eigen::Vector3d> ring2_filter;
static int ring2_filter_size = 7;
static bool is_arrive_ring2 = false;
static bool is_crossed_ring2 = false;
static int ring2_cross_count = 0;
static bool is_send_r2_postpoint = false;
static bool is_arrive_r2_postpoint = false;
static int ring2_approach_count = 0;
static int r2_postpoint_lock = 0;
static int r2_print_count = 0;

static double ring_start_time = 0;                  // 圆环感知开始时间
static double ring_end_time = 0;                    // 圆环感知结束时间

//double ring
static double doublering1_start_time = 0; 
static double doublering2_start_time = 0; 
static double doublering1_end_time = 0; 
static double doublering2_end_time = 0; 

//ap pre
static Eigen::Vector3d dynringpost_pose = {3.0, 0.0, 1.00};
static Eigen::Vector3d appre_pose = {4.8, 0.8, 1.00};

static Eigen::Vector3d double_pose1 = {-1.737, 3.69, 1.50};
static vector<Eigen::Vector3d> dou1_filter;
static int dou1_filter_size = 5;
static Eigen::Vector3d double_pose2 = {-1.687, 5.3, 1.50};
static vector<Eigen::Vector3d> dou2_filter;
static int dou2_filter_size = 5;

static bool getring1 = false;
static bool getring2 = false;


static int count1 = 0;
static int count2 = 0;


//apriltag
static Eigen::Vector3d ap_pose = {5.0, 1.1, 1.00};
static vector<Eigen::Vector3d> ap_filter;
static vector<Eigen::Vector3d> ap_his_pose;
static int ap_filter_size = 5;
static double ap_start_time = 0;
static double ap_end_time = 0;
static bool is_found_ap = false;
static int need_change_ap_target = false;
static int search_orient = 0;
static int ap_print_count = 0;
static bool ap_search_lock = false;
static int ap_search_lock_count = 0;
static int landing_count = 0;
static int predict_orient;
geometry_msgs::PoseStamped ap_gl_msg;
static Eigen::Vector3d ap_gl_pos;
static bool is_arrive_ap_prepoint = false;
static bool is_send_ap_prepoint = false;
static int ap_prepoint_lock = 0;
static int ctl_land_count = 0;
static bool land_success = false;
/*----------LXK----------*/


/**
 * @brief  set aim position
 * @param  _x_y_z position
 * @return NONE
 */
void Set_AimPos(double _x, double _y, double _z)
{
    aim_pos.pose.position.x = _x;
    aim_pos.pose.position.y = _y;
    aim_pos.pose.position.z = _z;
}

/**
 * @brief  set aim position
 * @param  _x_y_z position
 * @param  _yaw euler angle
 * @return NONE
 */
void Set_AimPos(double _x, double _y, double _z, double _yaw)
{
    aim_pos.pose.position.x = _x;
    aim_pos.pose.position.y = _y;
    aim_pos.pose.position.z = _z;
    Eigen::Quaterniond quat = EulerToQuat(0.0, 0.0, _yaw);
    aim_pos.pose.orientation.w = quat.w();
    aim_pos.pose.orientation.x = quat.x();
    aim_pos.pose.orientation.y = quat.y();
    aim_pos.pose.orientation.z = quat.z();
}


/**
 * @brief  set aim velocity
 * @param  _x_y_z velocity
 * @return NONE
 */
void Set_AimVel(double _x, double _y, double _z)
{
    aim_vel.twist.linear.x = _x;
    aim_vel.twist.linear.y = _y;
    aim_vel.twist.linear.z = _z;
}

/**
 * @brief  set aim velocity
 * @param  _x_y_z velocity
 * @param  _yaw body rate
 * @return NONE
 */
void Set_AimVel(double _x, double _y, double _z, double _yaw)
{
    aim_vel.twist.linear.x = _x;
    aim_vel.twist.linear.y = _y;
    aim_vel.twist.linear.z = _z;
    aim_vel.twist.angular.z = _yaw;
}


/**
 * @brief  set target according to type mask
 * @param  _x_y_z position / velocity / acceleration
 * @param  _yaw euler angle / body rate
 * @return NONE
 * @note   DO NOT use position + velocity + acceleration or yaw + yaw_rate together
 */
void Set_TargetPosition(double _x, double _y, double _z, double _yaw)
{
    target_pos.type_mask = // mavros_msgs::PositionTarget::IGNORE_PX |
                           // mavros_msgs::PositionTarget::IGNORE_PY |
                           // mavros_msgs::PositionTarget::IGNORE_PZ |
                           mavros_msgs::PositionTarget::IGNORE_VX |
                           mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::FORCE |
                           // mavros_msgs::PositionTarget::IGNORE_YAW |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target_pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // target_pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    
    target_pos.position.x = _x;
    target_pos.position.y = _y;
    target_pos.position.z = _z;
    // target_pos.velocity.x = _x;
    // target_pos.velocity.y = _y;
    // target_pos.velocity.z = _z;
    // target_pos.acceleration_or_force.x = _x;
    // target_pos.acceleration_or_force.y = _y;
    // target_pos.acceleration_or_force.z = _z;
    target_pos.yaw = _yaw;
    // target_pos.yaw_rate = _yaw;
}


/**
 * @brief  subscriber callback
 * @param  msg from estimator
 * @return NONE
 */
void Ready_Callback(const std_msgs::Bool::ConstPtr &msg)
{
    is_ready_fly = msg->data;
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 */
void Voltage_Callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage = msg->voltage;
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 */
void State_Callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 * @note   if initial w < 0, need to rotate yaw 360 deg
 */
void Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_fcu = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    quat_fcu = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    if(!is_quat_init)
    {
        is_quat_init = true;
        if(quat_fcu.w() < 0.0) {is_need_rot = true;}
        else {is_need_rot = false;}
    }
    if(is_need_rot) {quat_fcu = quat_fcu * Eigen::Quaterniond(-1.0, 0.0, 0.0, 0.0);}
    
    euler_fcu = QuatToEuler(quat_fcu);

    /*----------LXK----------*/
    if (!is_ini_time)
    {
        ini_time = msg->header.stamp;
        is_ini_time = true;
    }
    vector<double> fsm_pos_now(7);
    fsm_pos_now[0] = (msg->header.stamp-ini_time).toSec();//存放时间
    fsm_pos_now[1] = pos_fcu[0];
    fsm_pos_now[2] = pos_fcu[1];
    fsm_pos_now[3] = pos_fcu[2];
    fsm_pos_now[4] = euler_fcu[0];
    fsm_pos_now[5] = euler_fcu[1];
    fsm_pos_now[6] = euler_fcu[2];
    if (fsm_pos.size() < fsm_pos_num)
    {
        fsm_pos.push_back(fsm_pos_now);
    }
    else
    {
        for (int i=0;i<fsm_pos_num-1;i++)
        {
            fsm_pos[i] = fsm_pos[i+1];
        }
        fsm_pos.pop_back();
        fsm_pos.push_back(fsm_pos_now);
    }
    /*----------LXK----------*/
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 */
void Vel_Callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_fcu = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}


/**
 @brief  subscriber callback
 @param  msg from mavros
 @return NONE
 */
void IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    acc_imu = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    quat_imu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    euler_imu = QuatToEuler(quat_imu);
    rate_imu = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);   
}


/**
 @brief  subscriber callback
 @param  msg from mavros
 @return NONE
 */
void RC_Callback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    takeoff_channel = msg->channels[8];
    emergency_channel = msg->channels[10];
    ROS_INFO_THROTTLE(1.0, "emergency_channel=%d", emergency_channel);

    // cout << "takeoff_channel: " << takeoff_channel << endl;
}



/*--------------------------- Perception & Planning ---------------------------*/

CtrlPt traj_pt; // 规划轨迹
CtrlPt nmpc_traj_pt[10]; //nmpc 规划轨迹
Eigen::Vector3d nmpc_pos_des[10]; // 目标点
Eigen::Vector3d nmpc_vel_des[10]; // 目标速度

// SYX FSM TEST
traj_utils::Flag flag_traj_msg;//发布至ego，飞行点信息
traj_utils::Flag flag_state;//订阅至ego,运行状态信息反馈

//LYX ADD FOR SUPER
super_msgs::Flag flag_super_msg;//发布至super，飞行点信息
super_msgs::Flag flag_super_state;//接受自super，规划器运行状态

static bool dynamic_ready = false;
/*----------LXK----------*/
template <typename T>
bool MedianFilter(vector<T> &_filter, T _data, int _size, T &result)
{
    if (_filter.size() < _size-1)
    {
        _filter.push_back(_data);
        return false;
    }
    else if (_filter.size() < _size)
    {
        _filter.push_back(_data);
    }
    else
    {
        for (int i = 0; i < _size - 1; i++)
        {
            _filter[i] = _filter[i + 1];
        }
        _filter.pop_back();
        _filter.push_back(_data);
    }

    vector<T> sort_filter = _filter;
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.x() > b.x(); });
    result[0] = sort_filter[_size / 2][0];
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.y() > b.y(); });
    result[1] = sort_filter[_size / 2][1];
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.z() > b.z(); });
    result[2] = sort_filter[_size / 2][2];
    return true;
}

template <typename T>
bool MedAveFilter(vector<T> &_filter, T _data, int _size, T &result)
{
    if (_filter.size() < _size-1)
    {
        _filter.push_back(_data);
        return false;
    }
    else if (_filter.size() < _size)
    {
        _filter.push_back(_data);
    }
    else
    {
        for (int i = 0; i < _size - 1; i++)
        {
            _filter[i] = _filter[i + 1];
        }
        _filter.pop_back();
        _filter.push_back(_data);
    }

    vector<T> sort_filter = _filter;
    double sum0=0, sum1=0, sum2=0;
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.x() > b.x(); });
    for (int i=1;i<sort_filter.size()-1;i++)
    {
        sum0+=sort_filter[i][0];
    }
    result[0] = sum0/(_size-2);

    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.y() > b.y(); });
    for (int i=1;i<sort_filter.size()-1;i++)
    {
        sum1+=sort_filter[i][1];
    }
    result[1] = sum1/(_size-2);

    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.z() > b.z(); });
    for (int i=1;i<sort_filter.size()-1;i++)
    {
        sum2+=sort_filter[i][2];
    }
    result[2] = sum2/(_size-2);
    return true;
}

Eigen::Matrix4d LocalToGlobal(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
    Eigen::Matrix4d _LTG;
    _LTG(0, 0) = (cos(_pitch) * cos(_yaw));
    _LTG(0, 1) = (sin(_roll) * sin(_pitch) * cos(_yaw) - cos(_roll) * sin(_yaw));
    _LTG(0, 2) = (cos(_roll) * sin(_pitch) * cos(_yaw) + sin(_roll) * sin(_yaw));
    _LTG(1, 0) = (cos(_pitch) * sin(_yaw));
    _LTG(1, 1) = (sin(_roll) * sin(_pitch) * sin(_yaw) + cos(_roll) * cos(_yaw));
    _LTG(1, 2) = (cos(_roll) * sin(_pitch) * sin(_yaw) - sin(_roll) * cos(_yaw));
    _LTG(2, 0) = sin(_pitch);
    _LTG(2, 1) = sin(_roll) * cos(_pitch);
    _LTG(2, 2) = cos(_roll) * cos(_pitch);
    _LTG(0, 3) = _x;
    _LTG(1, 3) = _y;
    _LTG(2, 3) = _z;
    _LTG(3, 0) = 0;
    _LTG(3, 1) = 0;
    _LTG(3, 2) = 0;
    _LTG(3, 3) = 1;
    return _LTG;
}

bool JudgeDis(Eigen::Vector3d now, Eigen::Vector3d orin, double dis)
{
    if (fabs(now(0)-orin(0)) < dis && fabs(now(1)-orin(1)) < dis && fabs(now(2)-orin(2)) < dis)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool JudgeDis(Eigen::Vector3d now, Eigen::Vector3d orin, double _x, double _y, double _z)
{
    if (fabs(now(0)-orin(0)) < _x && fabs(now(1)-orin(1)) < _y && fabs(now(2)-orin(2)) < _z)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void PrintInfo(int &_count, int _size, string _info)
{
    if (_count < _size)
    {
        _count++;
    }
    else
    {
        cout << _info << endl;
        _count = 0;
    }
}

template<typename T>
void PrintInfo(int &_count, int _size, string _info, T data)
{
    if (_count < _size)
    {
        _count++;
    }
    else
    {
        cout << _info << data << endl;
        _count = 0;
    }
}
/*----------LXK----------*/


/**
 * @brief  规划轨迹回调函数 - 订阅自planner
 * @param
 * @return 无
 */
void Traj_Callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    // ROS_INFO("traj_received!");
    traj_pt.pos = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    traj_pt.vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    traj_pt.acc = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
}

/**
 * @brief  nmpc 规划轨迹回调函数 - 订阅自planner
 * @param
 * @return 无
 */
void nmpc_traj_cb(const traj_utils::Flag::ConstPtr &msg)
{
    // if (is_arrive_ap_prepoint)
    // {
    //     return;
    // }
    // if (task == 5 && is_send_r2_postpoint == false)
    // {
    //     return;
    // }
    is_get_planner_msgs = true;

    for(int i=0; i<6; i++)
    {
        nmpc_traj_pt[i].pos = Eigen::Vector3d(msg->cmd[i].position.x, msg->cmd[i].position.y, msg->cmd[i].position.z);
        nmpc_traj_pt[i].vel = Eigen::Vector3d(msg->cmd[i].velocity.x, msg->cmd[i].velocity.y, msg->cmd[i].velocity.z);
        nmpc_traj_pt[i].acc = Eigen::Vector3d(msg->cmd[i].acceleration.x, msg->cmd[i].acceleration.y, msg->cmd[i].acceleration.z);
    }

    // //wsz
    // for(int i=0; i<6; i++)
    // {
    //     nmpc_traj_pt[i].pos = Eigen::Vector3d(msg->cmd[i].position.x, msg->cmd[i].position.y, 1.0);
    //     nmpc_traj_pt[i].vel = Eigen::Vector3d(msg->cmd[i].velocity.x, msg->cmd[i].velocity.y, msg->cmd[i].velocity.z);
    //     nmpc_traj_pt[i].acc = Eigen::Vector3d(0.0, 0.0, 0.0);
    //     cout << i << ":" << nmpc_traj_pt[i].pos[0] << ", " << nmpc_traj_pt[i].pos[1] << endl;
    // }
    // //wsz

    // vector<CtrlPt> _traj_nmpc;
    // for(int i = 0; i < 6; i++)
    // {
    //     CtrlPt _pt;
    //     _pt.pos = Eigen::Vector3d(msg->cmd[i].position.x, msg->cmd[i].position.y, msg->cmd[i].position.z);
    //     _pt.vel = Eigen::Vector3d(msg->cmd[i].velocity.x, msg->cmd[i].velocity.y, msg->cmd[i].velocity.z);
    //     _pt.acc = Eigen::Vector3d(msg->cmd[i].acceleration.x, msg->cmd[i].acceleration.y, msg->cmd[i].acceleration.z);
    //     _pt.euler(2) = msg->cmd[i].yaw;
    //     _traj_nmpc.push_back(_pt);
    // }
    // traj_nmpc = _traj_nmpc;
}
void nmpc_super_cb(const super_msgs::Flag::ConstPtr &msg)
{
    //lyx for super
    for(int i=0; i<6; i++)
    {
        nmpc_traj_pt[i].pos = Eigen::Vector3d(msg->cmd[i].position.x, msg->cmd[i].position.y, msg->cmd[i].position.z);
        nmpc_traj_pt[i].vel = Eigen::Vector3d(msg->cmd[i].velocity.x, msg->cmd[i].velocity.y, msg->cmd[i].velocity.z);
        nmpc_traj_pt[i].acc = Eigen::Vector3d(msg->cmd[i].acceleration.x, msg->cmd[i].acceleration.y, msg->cmd[i].acceleration.z);
    }
}

/**
 * @brief  nmpc 规划轨迹回调函数 - 订阅自planner
 * @param
 * @return 无
 */
void PlannerCallback(const super_msgs::Flag::ConstPtr &msg)
{
    if(is_get_planner_msgs == false)
    {is_get_planner_msgs = true;}

    for(int i=0; i<9; i++)
    {
        nmpc_pos_des[i] = Eigen::Vector3d(msg->cmd[i].position.x, msg->cmd[i].position.y, msg->cmd[i].position.z);
        nmpc_vel_des[i] = Eigen::Vector3d(msg->cmd[i].velocity.x, msg->cmd[i].velocity.y, msg->cmd[i].velocity.z);
    }
}



/**
 @brief  subscriber callback
 @param  msg from odometry
 @return NONE
 */
void Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_odom = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vel_odom = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    quat_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_odom = QuatToEuler(quat_odom);
}

/*----------LXK----------*/
void ring_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ring_start_time = msg->header.stamp.toSec();
    cout << "\033[33m" << "Ring callback!" << endl;
    cout << "Ring Delta Time: " << ring_start_time - ring_end_time << "\033[0m" << endl;
    ring_end_time = ring_start_time;                       //Monitor the time between two frame

    Eigen::Vector3d prepose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    cout << "dynamic ring: [" << ring2_pose(0) << ", " << ring2_pose(1) << ", " << ring2_pose(2) << "]" << endl;
    
    dynamic_ring == true;
   // cout << "have found dynamic ring" << endl;

    
    
        if (JudgeDis(prepose, ring2_pose, 1.0))
        {
            Eigen::Vector3d filterpose;
            if (MedAveFilter(ring2_filter, prepose, ring2_filter_size, filterpose))   //这里是平均中值
            {
                // if (task == 5)
                // {
                    // ring2_pose = filterpose;
                    // cout << "dynamic_ring seted !" << endl;
                    // cout << "pos: [" << ring2_pose(0) << ", " << ring2_pose(1) << ", " << ring2_pose(2) << "]" << "\033[0m" << endl;
                    // set_dynamic_ring = true;
                    // need_GeneTraj = true;
                // }
                // else
                // {
                    if (!JudgeDis(filterpose, ring2_pose, 0.10))
                    {
                        cout << "\033[32m" << "ring2 changed!" << endl;
                        cout << "ring2 changed!" << endl;
                        cout << "ring2 changed!" << endl;
                        cout << "From: [" << ring2_pose(0) << ", " << ring2_pose(1) << ", " << ring2_pose(2) << "]" << endl;
                        cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                        ring2_pose = filterpose;
                        need_GeneTraj = true;
                        static bool dynamic_ring = true;
                    }
                    else
                    {
                        cout << "No need to change ring2!!!" << endl;
                    }
                // }
            }
            else
            {
                cout << "ring2 Filter is not full!" << endl;
            }
        }
        else
        {
            cout << "Perception of ring2 is out of security range!" << endl;
        }
   
    
}

void double_ring1_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    doublering1_start_time = msg->header.stamp.toSec();
    cout << "\033[33m" << "doubleRing1 callback!" << endl;
    cout << "doubleRing1 Delta Time: " << doublering1_start_time - doublering1_end_time << "\033[0m" << endl;
    doublering1_end_time = doublering1_start_time;                       //Monitor the time between two frame

    Eigen::Vector3d prepose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    cout << "double1: [" << double_pose1(0) << ", " << double_pose1(1) << ", " << double_pose1(2) << "]" << endl;
    
        if (JudgeDis(prepose, double_pose1, 1.0))
        {
            Eigen::Vector3d filterpose;
            if (MedianFilter(dou1_filter, prepose, dou1_filter_size, filterpose))
            {
                if (!JudgeDis(filterpose, double_pose1, 0.10))
                {
                    cout << "\033[32m" << "ring1 changed!" << endl;
                    cout << "duoblering1 changed!" << endl;
                    cout << "doublering1 changed!" << endl;
                    cout << "From: [" << double_pose1(0) << ", " << double_pose1(1) << ", " << double_pose1(2) << "]" << endl;
                    cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                    double_pose1 = filterpose;
                    need_GeneTraj = true;
                }
                else
                {
                    cout << "No need to change doublering1!!!" << endl;
                }
            }
            else
            {
                cout << "doublering1 Filter is not full!" << endl;
            }
        }
        else
        {
            cout << "Perception of doublering1 is out of security range!" << endl;
        }
    
}

void double_ring2_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    doublering2_start_time = msg->header.stamp.toSec();
    cout << "\033[33m" << "doubleRing2 callback!" << endl;
    cout << "doubleRing2 Delta Time: " << doublering2_start_time - doublering2_end_time << "\033[0m" << endl;
    doublering2_end_time = doublering2_start_time;                       //Monitor the time between two frame
    cout << "double2: [" << double_pose2(0) << ", " << double_pose2(1) << ", " << double_pose2(2) << "]" << endl;
    Eigen::Vector3d prepose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
        if (JudgeDis(prepose, double_pose2, 1.0))
        {
            Eigen::Vector3d filterpose;
            if (MedianFilter(dou2_filter, prepose, dou2_filter_size, filterpose))
            {
                if (!JudgeDis(filterpose, double_pose2, 0.10))
                {
                    cout << "\033[32m" << "doublering2 changed!" << endl;
                    cout << "doublering2 changed!" << endl;
                    cout << "doublering2 changed!" << endl;
                    cout << "From: [" << double_pose2(0) << ", " << double_pose2(1) << ", " << double_pose2(2) << "]" << endl;
                    cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                    double_pose2 = filterpose;
                    need_GeneTraj = true;
                }
                else
                {
                    cout << "No need to change doublering2!!!" << endl;
                }
            }
            else
            {
                cout << "doublering2 Filter is not full!" << endl;
            }
        }
        else
        {
            cout << "Perception of doublering2 is out of security range!" << endl;
        }
    
}

void tunnel_cb(const nav_msgs::Path::ConstPtr &msg)
{
    Eigen::Vector3d prepose(msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z);
    cout << "tunnel1: [" << tunnel_pose1(0) << ", " << tunnel_pose1(1) << ", " << tunnel_pose1(2) << "]" << endl;
    if (JudgeDis(prepose, tunnel_pose1, 1.0))
    {
        Eigen::Vector3d filterpose;
        if (MedianFilter(tunnel_filter1, prepose, tunnel_filter_size, filterpose))
        {
            if (!JudgeDis(filterpose, tunnel_pose1, 0.10))
            {
                cout << "\033[32m" << "tunnel changed!" << endl;
                cout << "tunnel changed!" << endl;
                cout << "tunnel changed!" << endl;
                cout << "From: [" << tunnel_pose1(0) << ", " << tunnel_pose1(1) << ", " << tunnel_pose1(2) << "]" << endl;
                cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                tunnel_pose1 = filterpose;
                need_GeneTraj = true;
            }
            else
            {
                cout << "No need to change tunnel1!!!" << endl;
            }
        }
        else
        {
            cout << "tunnel Filter is not full!" << endl;
        }
    }
    else
    {
        cout << "Perception of tunnel is out of security range!" << endl;
    }
    
    prepose << msg->poses[1].pose.position.x, msg->poses[1].pose.position.y, msg->poses[1].pose.position.z;
    cout << "tunnel2: [" << tunnel_pose2(0) << ", " << tunnel_pose2(1) << ", " << tunnel_pose2(2) << "]" << endl;
    if (JudgeDis(prepose, tunnel_pose2, 1.0))
    {
        Eigen::Vector3d filterpose;
        if (MedianFilter(tunnel_filter2, prepose, tunnel_filter_size, filterpose))
        {
            if (!JudgeDis(filterpose, tunnel_pose2, 0.10))
            {
                cout << "\033[32m" << "tunnel2 changed!" << endl;
                cout << "tunnel changed!" << endl;
                cout << "tunnel changed!" << endl;
                cout << "From: [" << tunnel_pose2(0) << ", " << tunnel_pose2(1) << ", " << tunnel_pose2(2) << "]" << endl;
                cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                tunnel_pose2= filterpose;
                need_GeneTraj = true;
            }
            else
            {
                cout << "222No need to change tunnel2!!!" << endl;
            }
        }
        else
        {
            cout << "222tunnel Filter is not full!" << endl;
        }
    }
    else
    {
        cout << "222Perception of tunnel2 is out of security range!" << endl;
    }

    prepose << msg->poses[2].pose.position.x, msg->poses[2].pose.position.y, msg->poses[2].pose.position.z;
    cout << "tunnel3: [" << tunnel_pose3(0) << ", " << tunnel_pose3(1) << ", " << tunnel_pose3(2) << "]" << endl;
    if (JudgeDis(prepose, tunnel_pose3, 1.0))
    {
        Eigen::Vector3d filterpose;
        if (MedianFilter(tunnel_filter3, prepose, tunnel_filter_size, filterpose))
        {
            if (!JudgeDis(filterpose, tunnel_pose3, 0.10))
            {
                cout << "\033[32m" << "tunnel3 changed!" << endl;
                cout << "tunnel3 changed!" << endl;
                cout << "tunnel3 changed!" << endl;
                cout << "From: [" << tunnel_pose3(0) << ", " << tunnel_pose3(1) << ", " << tunnel_pose3(2) << "]" << endl;
                cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                tunnel_pose3= filterpose;
                need_GeneTraj = true;
            }
            else
            {
                cout << "333No need to change tunnel3!!!" << endl;
            }
        }
        else
        {
            cout << "333tunnel Filter is not full!" << endl;
        }
    }
    else
    {
        cout << "333Perception of tunnel3 is out of security range!" << endl;
    }
}

void maze_cb(const nav_msgs::Path::ConstPtr &msg)
{
    Eigen::Vector3d prepose(msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z);
    cout << "maze: [" << maze_pose1(0) << ", " << maze_pose1(1) << ", " << maze_pose1(2) << "]" << endl;
    if (JudgeDis(prepose, maze_pose1, 1.0))
    {
        Eigen::Vector3d filterpose;
        if (MedianFilter(maze_filter1, prepose, maze_filter_size, filterpose))
        {
            if (!JudgeDis(filterpose, maze_pose1, 0.10))
            {
                cout << "\033[32m" << "maze changed!" << endl;
                cout << "maze changed!" << endl;
                cout << "maze changed!" << endl;
                cout << "From: [" << maze_pose1(0) << ", " << maze_pose1(1) << ", " << maze_pose1(2) << "]" << endl;
                cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                maze_pose1 = filterpose;
                need_GeneTraj = true;
            }
            else
            {
                cout << "No need to change maze!!!" << endl;
            }
        }
        else
        {
            cout << "maze Filter is not full!" << endl;
        }
    }
    else
    {
        cout << "Perception of maze is out of security range!" << endl;
    }
    
    prepose << msg->poses[1].pose.position.x, msg->poses[1].pose.position.y, msg->poses[1].pose.position.z;
    cout << "maze: [" << maze_pose2(0) << ", " << maze_pose2(1) << ", " << maze_pose2(2) << "]" << endl;
    if (JudgeDis(prepose, maze_pose2, 1.0))
    {
        Eigen::Vector3d filterpose;
        if (MedianFilter(maze_filter2, prepose, maze_filter_size, filterpose))
        {
            if (!JudgeDis(filterpose, maze_pose2, 0.10))
            {
                cout << "\033[32m" << "maze changed!" << endl;
                cout << "222maze changed!" << endl;
                cout << "222maze changed!" << endl;
                cout << "From: [" << maze_pose2(0) << ", " << maze_pose2(1) << ", " << maze_pose2(2) << "]" << endl;
                cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                maze_pose2 = filterpose;
                need_GeneTraj = true;
            }
            else
            {
                cout << "222No need to change maze!!!" << endl;
            }
        }
        else
        {
            cout << "222maze Filter is not full!" << endl;
        }
    }
    else
    {
        cout << "222Perception of maze is out of security range!" << endl;
    }    
    
    prepose << msg->poses[2].pose.position.x, msg->poses[2].pose.position.y, msg->poses[2].pose.position.z;
    cout << "maze: [" << maze_pose3(0) << ", " << maze_pose3(1) << ", " << maze_pose3(2) << "]" << endl;
    if (JudgeDis(prepose, maze_pose3, 1.0))
    {
        Eigen::Vector3d filterpose;
        if (MedianFilter(maze_filter3, prepose, maze_filter_size, filterpose))
        {
            if (!JudgeDis(filterpose, maze_pose3, 0.10))
            {
                cout << "\033[32m" << "maze changed!" << endl;
                cout << "333maze changed!" << endl;
                cout << "333maze changed!" << endl;
                cout << "From: [" << maze_pose3(0) << ", " << maze_pose3(1) << ", " << maze_pose3(2) << "]" << endl;
                cout << "To: [" << filterpose(0) << ", " << filterpose(1) << ", " << filterpose(2) << "]" << "\033[0m" << endl;
                maze_pose3 = filterpose;
                need_GeneTraj = true;
            }
            else
            {
                cout << "333No need to change maze!!!" << endl;
            }
        }
        else
        {
            cout << "333maze Filter is not full!" << endl;
        } 
    }
    else
    {
        cout << "333Perception of maze is out of security range!" << endl;
    }
}


void apriltag_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)    // 0705
{
    // if (pos_fcu(2) < 0.2)
    // {
    //     ROS_ERROR("Z is too low!!! Return!!!");
    //     return;
    // }
    if(ctl_land_count<230)
    {
        ap_start_time = (msg->header.stamp-ini_time).toSec();
        cout << "\033[33m" << "ap callback!" << endl;
        cout << "ap Delta Time: " << ap_start_time - ap_end_time << "\033[0m" << endl;
        ap_end_time = ap_start_time;                //Monitor the time between two frame

        Eigen::Vector4d prepose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 1);
        cout << "prepose: " << prepose.transpose() << endl;
        double min_delta_time = 100000;
        int num = 0;
        for (int i=0;i<fsm_pos.size();i++)
        {
            double delta_time = fabs(ap_start_time - fsm_pos[i][0]);
            if (delta_time < min_delta_time)
            {
                min_delta_time = delta_time;
                num = i;
            }
        }
        Eigen::Matrix4d m;
        m << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        prepose = m * prepose;
        Eigen::Vector4d LTGpose = LocalToGlobal(fsm_pos[num][1], fsm_pos[num][2], fsm_pos[num][3], 
                                                fsm_pos[num][4], fsm_pos[num][5], fsm_pos[num][6]) * prepose;
        Eigen::Vector3d LTGpose03 = LTGpose.head(3);

        ap_gl_pos = LTGpose03;
        // cout << "LTGpose03: [" << LTGpose03(0) << ", " << LTGpose03(1) << ", " << LTGpose03(2) << "]" << endl;

        if (JudgeDis(LTGpose03, ap_pose, 2.0))
        {
            Eigen::Vector3d filterpose;
            if (MedianFilter(ap_filter, LTGpose03, ap_filter_size, filterpose))
            {
                if (is_found_ap == false)
                {
                    is_found_ap = true;
                    ROS_WARN("Apriltag has been found!!!");
                    ROS_WARN("Apriltag has been found!!!");
                    ROS_WARN("Apriltag has been found!!!");
                }
                ap_pose = filterpose;
                april_pos.header.stamp = ros::Time::now();
                april_pos.header.frame_id = "map"; // 根据实际情况修改坐标系

                // 设置位置信息
                april_pos.pose.position.x = ap_pose(0);
                april_pos.pose.position.y = ap_pose(1);
                april_pos.pose.position.z = ap_pose(2);

                // 设置姿态为单位四元数（如果不需要姿态信息）
                april_pos.pose.orientation.x = 0.0;
                april_pos.pose.orientation.y = 0.0;
                april_pos.pose.orientation.z = 0.0;
                april_pos.pose.orientation.w = 1.0;

                cout << "LTGpose03: [" << ap_pose(0) << ", " << ap_pose(1) << ", " << ap_pose(2) << "]" << endl;
            }
            else
            {
                cout << "Apriltag filter is not full!" << endl;
            }
        }
        else
        {
            cout << "Apriltag is out of security range!" << endl;
            cout << "[" << LTGpose03.x() << ", " << LTGpose03.y() << ", " << LTGpose03.z() << "]" << endl;
        }
    }
    
}

// void YawSmooth(double &yaw_now, double yaw_des)
// {
//     if (yaw_now < yaw_des - 0.02)
//     {
//         yaw_now += 0.01;
//     }
//     else if (yaw_now > yaw_des + 0.02)
//     {
//         yaw_now -= 0.01;
//     }
// }

void EgoAddPoint(int _id, int _mode, int _is_map, double _x, double _y, double _z, double _yaw, double _perc_mode)
{
    TypePoint pose;
    pose.id = _id;
    pose.mode = _mode;
    pose.is_map = _is_map;
    pose.x = _x;
    pose.y = _y;
    pose.z = _z;
    pose.yaw = _yaw;
    pose.perc_mode = _perc_mode;
    Ego_traj.push_back(pose);
}

// 存放所有的轨迹点，0为id，1为mode，2为map building, 3为x，4为y，5为z，6为yaw，7为感知模式
void EgoGeneTraj()
{
    Ego_traj.clear();
    // EgoAddPoint(0, 2, 0, 2.36, 0.23, 1.25, 0.0, 7);
    // EgoAddPoint(1, 3, 0, 3.35, 3.2, 1.25, 1.57, 7);
    // EgoAddPoint(2, 3, 0, 1.57, 4.47, 1.25, 3.13, 7);
    // EgoAddPoint(3, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.50, 3.13, 0);
    // EgoAddPoint(4, 3, 0, double_pose2(0), double_pose2(1), 1.50, 3.13, 0);
    // EgoAddPoint(5, 3, 0, double_pose2(0)-0.8, double_pose2(1), 1.50, 3.13, 0);
    // EgoAddPoint(6, 3, 0, double_pose1(0)-0.8, double_pose1(1), 1.50, 3.13, 0);
    // EgoAddPoint(7, 3, 0, double_pose1(0), double_pose1(1), 1.50, 3.13, 0);
    // EgoAddPoint(8, 3, 0, double_pose1(0)+1.8, double_pose1(1), 1.50, 3.13, 0);
    // EgoAddPoint(9, 3, 0, double_pose1(0)+1.8, double_pose1(1)-1, 0.5, 3.13, 0);


    
    EgoAddPoint(0, 2, 0, 1.35, 0.0, 1.08, 0.0, 3);
    EgoAddPoint(1, 3, 0, 3.2, 0.09, 1.85, 0.0, 0);
    EgoAddPoint(2, 3, 0, 5.3, 0.195, 1.85, 0.0, 0);
    EgoAddPoint(3, 3, 0, 6.28, 0.902, 1.85, 0.0, 0);
    EgoAddPoint(4, 3, 0, 7.16, 0.95, 1.85, 0.0, 0);
    EgoAddPoint(5, 3, 0, 7.68, 0.16, 1.85, 0.0, 0);
    EgoAddPoint(6, 3, 0, 9.15, 0.35, 1.85, 0.0, 0);
    EgoAddPoint(7, 3, 0, 10.32, 0.53, 1.85, 0.0, 0);
    EgoAddPoint(8, 3, 0, 12.32, 0.53, 1.85, 0.0, 2);
    EgoAddPoint(9, 3, 0, 12.89, 0.05, 1.85, 0.0, 2);
    EgoAddPoint(10, 3, 0, 13.12, -0.48, 1.85, 0.0, 2);
    EgoAddPoint(11, 3, 0, tunnel_pose1(0)-0.8, tunnel_pose1(1), 1.75, 0.0, 0);
    EgoAddPoint(12, 3, 0, tunnel_pose1(0), tunnel_pose1(1), 1.75, 0.0, 0);
    EgoAddPoint(13, 3, 0, tunnel_pose1(0)+0.8, tunnel_pose1(1), 1.85, 0.0, 0);
    EgoAddPoint(14, 3, 0, tunnel_pose3(0)-0.5, tunnel_pose3(1), 1.85, 0.0, 0);
    EgoAddPoint(15, 3, 0, tunnel_pose3(0), tunnel_pose3(1), 1.85, 0.0, 0);
    EgoAddPoint(16, 3, 0, tunnel_pose3(0)+0.5, tunnel_pose3(1), 1.85, 0.0, 0);
    EgoAddPoint(17, 3, 0, tunnel_pose2(0)-1.0, tunnel_pose2(1), 1.85, 0.0, 0);
    EgoAddPoint(18, 3, 0, tunnel_pose2(0), tunnel_pose2(1), 1.85, 0.0, 0);
    EgoAddPoint(19, 3, 0, tunnel_pose2(0)+1.0, tunnel_pose2(1), 1.85, 0.0, 0);
    EgoAddPoint(20, 3, 0, 24.65, 0.85, 1.8, 0.0, 0);
    EgoAddPoint(21, 3, 0, 20.80, 7.55, 2.0, 1.57, 0);
    EgoAddPoint(22, 3, 0, 19.85, 8.34, 2.0, 1.57, 3);
    EgoAddPoint(23, 2, 0, maze_pose1(0), maze_pose1(1)-0.8, 2.0, 1.57, 0);
    EgoAddPoint(24, 2, 0, maze_pose1(0), maze_pose1(1), 2.0, 1.57, 0);
    EgoAddPoint(25, 2, 0, maze_pose1(0), maze_pose1(1)+0.8, 2.0, 1.57, 0);
    EgoAddPoint(26, 2, 0, maze_pose2(0), maze_pose2(1)-0.8, 2.0, 1.57, 0);
    EgoAddPoint(27, 2, 0, maze_pose2(0), maze_pose2(1), 2.0, 1.57, 0);
    EgoAddPoint(28, 2, 0, maze_pose2(0), maze_pose2(1)+0.8, 2.0, 1.57, 0);
    EgoAddPoint(29, 2, 0, maze_pose3(0), maze_pose3(1)-0.8, 2.0, 1.57, 0);
    EgoAddPoint(30, 2, 0, maze_pose3(0), maze_pose3(1), 2.0, 1.57, 0);
    EgoAddPoint(31, 2, 0, maze_pose3(0), maze_pose3(1)+0.8, 2.0, 1.57, 0);
    EgoAddPoint(32, 3, 0, 19.00, 18.00, 1.8, 1.57, 0);
    EgoAddPoint(33, 3, 0, 14.80, 15.42, 1.0, 3.13, 0);
    EgoAddPoint(34, 2, 0, 2.479, 0.0, 1.0, 0.0, 7);
    EgoAddPoint(34, 3, 0, 3.26, 4.0, 1.65, 1.57, 7);
    EgoAddPoint(36, 3, 0, 1.12, 4.37, 1.65, 3.13, 7);
    EgoAddPoint(37, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.65, 3.13, 0);
    EgoAddPoint(38, 3, 0, double_pose2(0), double_pose2(1), 1.65, 3.13, 0);
    EgoAddPoint(39, 3, 0, double_pose2(0)-0.8, double_pose2(1), 1.65, 3.13, 0);
    EgoAddPoint(40, 3, 0, double_pose1(0)-0.8, double_pose1(1), 1.65, 3.13, 0);
    EgoAddPoint(41, 3, 0, double_pose1(0), double_pose1(1), 1.65, 3.13, 0);
    EgoAddPoint(42, 3, 0, double_pose1(0)+1.8, double_pose1(1), 1.65, 3.13, 0);
    EgoAddPoint(43, 3, 0, double_pose1(0)+1.8, double_pose1(1)-1, 1.65, 3.13, 0);
    EgoAddPoint(44, 3, 0, -4.3, 2.12, 1.65, 3.13, 5);
    EgoAddPoint(45, 3, 0, ring2_pose(0)+1,ring2_pose(1), 1.65, 3.13, 5);
    
    // EgoAddPoint(8, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.65, 0.0, 0);
    // EgoAddPoint(9, 3, 0, double_pose2(0)-0.8, double_pose2(1), 1.65, 0.0, 0);
    // EgoAddPoint(9, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.65, 0.0, 0);
    // EgoAddPoint(10, 2, 0, ring2_pose(0)-1,ring2_pose(1), ring2_pose(2), 0.0, 5);
    // EgoAddPoint(2, 3, 0, 10.9, -0.12, 1.2, 0.0, 5);
    // EgoAddPoint(3, 3, 0, 11.2, -1.1, 1.2, -3.13, 5);
    // EgoAddPoint(4, 3, 0, 9.66, -1.35, 1.4, -3.13, 5);
    // EgoAddPoint(5, 3, 0, 8.77, -1.35, 1.4, -3.13, 5);
    // EgoAddPoint(6, 3, 0, 7.9, -4.7, 1.3, -3.13, 5);
    // EgoAddPoint(7, 3, 0, double_pose1(0)+1, double_pose1(1), 1.65, -3.13, 5);
    // EgoAddPoint(8, 3, 0, double_pose1(0)-1, double_pose1(1), 1.65, -3.13, 5);
    // EgoAddPoint(9, 3, 0, double_pose2(0)-1, double_pose2(1), 1.65, -3.13, 5);
    // EgoAddPoint(10, 3, 0, double_pose2(0)+1, double_pose2(1), 1.65, -3.13, 5);
    // EgoAddPoint(11, 3, 0, 7.49, -5.48, 0.5, -3.13, 5);

    // EgoAddPoint(0, 2, 0, 3, 0, 0.5, 0.0, 5);
    // EgoAddPoint(1, 2, 0, double_pose1(0)-0.5, double_pose1(1), double_pose1(2), 0.0, 5);
    // EgoAddPoint(2, 2, 0, double_pose1(0)+0.5, double_pose1(1), double_pose1(2), 0.0, 5);
    // EgoAddPoint(3, 2, 0, double_pose2(0)+0.5, double_pose2(1), double_pose2(2), 0.0, 5);  //forest
    // EgoAddPoint(4, 2, 0, double_pose2(0)-0.5, double_pose2(1), double_pose2(2), 0.0, 5);
    // EgoAddPoint(4, 2, 0, tunnel_pose1(0), tunnel_pose1(1), tunnel_pose1(2), 0.0, 5);
    // EgoAddPoint(5, 2, 0, tunnel_pose2(0), tunnel_pose2(1), tunnel_pose2(2), 0.0, 4);
    // EgoAddPoint(6, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);  //tunnel
    // EgoAddPoint(7, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);
    // EgoAddPoint(8, 2, 0, maze_pose1(0), maze_pose1(1), maze_pose1(2), 0.0, 6);
    // EgoAddPoint(9, 2, 0, maze_pose2(0)+1, maze_pose2(1), maze_pose2(2), 1.57, 6);
    // EgoAddPoint(10, 2, 0, maze_pose3(0)+1, maze_pose3(1)+1, maze_pose3(2), 3.14, 6);
    // EgoAddPoint(11, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);  //maze
    // EgoAddPoint(12, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);
    // EgoAddPoint(13, 2, 0, ring1_pose(0), ring1_pose(1), ring1_pose(2), 3.14, 4);
    // EgoAddPoint(14, 2, 0, ring1_pose(0)+2, ring1_pose(1), ring1_pose(2), 3.14, 5);
    // EgoAddPoint(15, 2, 0, ring2_pose(0)+2, ring2_pose(1), ring2_pose(2), 3.14, 4);
    // EgoAddPoint(16, 2, 0, ring2_pose(0), ring2_pose(1), ring2_pose(2), 3.14, 5);
    // EgoAddPoint(17, 2, 0, double_pose1(0), double_pose1(1), double_pose1(2), 3.14, 5);  //double ring
    // EgoAddPoint(18, 2, 0, double_pose2(0), double_pose2(1), double_pose2(2), 3.14, 5);  
    // EgoAddPoint(19, 2, 0, ring3_pose(0), ring3_pose(1), ring3_pose(2), 3.14, 5);  //dynamic ring
}

/*----------LXK----------*/




/**
 * @date 2024-04-11 03:10:00 
 * @author SHI YANGXI
 * @brief  New Ego_traj function , not include pub function!
 * @param id:   id of each waypoint, should start from 0 and increase by 1 each time
 * @param mode :    0 : init mode; 1: normal mode; 2: slow mode; 3: stop mode(not used)  // not finished and tested yet
 * @param x:    position x,double
 * @param y:    position y,double
 * @param z:    position z,double
 * @param  traj_utils::Flag , should be a ptr of flag msg
 * @param  super_msgs::Flag , should be a ptr of flag msg
 * @return none
 */
void EGO_flag_aimpos(TypePoint point)
{
    flag_traj_msg.header.frame_id = "world";
    flag_traj_msg.header.stamp = ros::Time::now();
    flag_traj_msg.header.seq = 0;
    flag_traj_msg.id = point.id;
    flag_traj_msg.mode = point.mode;
    flag_traj_msg.is_map = point.is_map;
    flag_traj_msg.position.x = point.x;
    flag_traj_msg.position.y = point.y;
    flag_traj_msg.position.z = point.z;

    flag_super_msg.header.frame_id = "world";
    flag_super_msg.header.stamp = ros::Time::now();
    flag_super_msg.header.seq = 0;
    flag_super_msg.id = point.id;
    flag_super_msg.mode = point.mode;
    flag_super_msg.is_map = point.is_map;
    flag_super_msg.position.x = point.x;
    flag_super_msg.position.y = point.y;
    flag_super_msg.position.z = point.z;
}
/**
 * @date 2024-04-11 03:10:00
 * @author SHI YANGXI
 * @brief  New Ego_traj positionCommand callback function
 * @param  quadrotor_msgs::PositionCommand::ConstPtr &msg : the ptr of control message
 * @return none
 */


/**
 * @date 2024-04-13 19:10:00
 * @author SHI YANGXI
 * @brief  New Ego_traj state callback function
 * @param  traj_utils::FlagConstPtr &msg : the ptr of ego state message
 * @param  super_msgs::FlagConstPtr &msg : the ptr of ego state message
 * @return none
 */
void EGO_flag_state_cb(const traj_utils::FlagStateConstPtr &msg)
{   
    flag_state.now_id = msg->now_id; 
    flag_state.touch_goal = msg->touch_goal;
    
}
void SUPER_flag_state_cb(const super_msgs::FlagConstPtr &msg)
{   
    flag_super_state.now_id = msg->now_id; 
    flag_super_state.touch_goal = msg->touch_goal;
}

/*--------------------------- Main ---------------------------*/

void UdpListen(const uint16_t cport)
{
    ros::NodeHandle nh;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        ROS_ERROR("Network Error!!!");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_lis;
    int len;
    memset(&addr_lis, 0, sizeof(struct sockaddr_in));
    addr_lis.sin_family = AF_INET;
    addr_lis.sin_port = htons(cport);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY); // 自动获取IP地址
    len = sizeof(addr_lis);

    /* 绑定socket */
    if (bind(sock_fd, (struct sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
        perror("bind error:");
        exit(1);
    }

    int recv_num;
    char recv_buf[100];
    const char dot[2] = ",";
    struct sockaddr_in addr_client;

    while (ros::ok())
    {
        char *p;
        int ent = 0;
        int user_cmd;
        double px, py, pz;

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if (recv_num < 0 || abs(recv_num - 19) > 3)
        {
            ROS_ERROR("Receive Fail!!!");
            continue;
        }
        recv_buf[recv_num] = '\0';
        // ROS_INFO("Rec: %s, len = %d", recv_buf, recv_num);

        /* receive UDP data */
        p = strtok(recv_buf, dot);
        sscanf(p, "%d", &user_cmd);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &px);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &py);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &pz);
        
        if (is_udp_enable) {cmd = user_cmd;}
        ROS_INFO_THROTTLE(1.0, "is_udp_enable=%d", static_cast<int>(is_udp_enable));
        ROS_INFO_THROTTLE(1.0, "user_cmd=%d", user_cmd);

        /*    wait for pose EKF    */
        if (cmd == 0)
        {
            //if(is_ready_fly) {ROS_INFO("Ready Fly");}
            //else {ROS_WARN("Waiting Pose EKF!");}
        }
        /* low battery protection (land and disable UDP) */
        else
        {
            // if(voltage < 14.8)
            // {
            //     cmdd = 4;
            //     flag_udp = false;
            //     ROS_ERROR("Low Battery!!!");
            // }
            // if(emergency_channel > 1500)
            // // ROS_INFO_THROTTLE(1.0, "emergency_channel=%d", emergency_channel);
            // {cmd = 9;}
        }

    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle nh;

    /*--------- Publisher ---------*/
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_target_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10);
    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher vis_att_pub = nh.advertise<geometry_msgs::PoseStamped>("/vis", 100);
    ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_goal", 10);
    // SYX FSM TEST
    ros::Publisher ego_flag_pub = nh.advertise<traj_utils::Flag>("/ego_planner/flag_msg", 10);
    // SYX FSM TEST DONE
    ros::Publisher nmpc_posref_pub = nh.advertise<geometry_msgs::PoseStamped>("/nmpc_posref", 10);
    ros::Publisher nmpc_posfdb_pub = nh.advertise<geometry_msgs::PoseStamped>("/nmpc_posfdb", 10);
    /*----------LXK----------*/
    ros::Publisher perc_mode_pub = nh.advertise<std_msgs::Int64>("/perc_mode", 10);
    ros::Publisher ap_pub = nh.advertise<geometry_msgs::PoseStamped>("/ap_global", 10);
    // ros::Publisher ring_pose_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/ring_pose1", 10);
    // ros::Publisher ring_pose_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/ring_pose2", 10);
    ros::Publisher nmpc_state_pub = nh.advertise<fsm_ctrl::nmpc_state>("/nmpc_state", 10);
    ros::Publisher traj_path_pub = nh.advertise<nav_msgs::Path>("/traj_ref_path", 10);
    /*----------LXK----------*/

    /*--------- Subscriber ---------*/
    ros::Subscriber ready_sub = nh.subscribe<std_msgs::Bool>
        ("/fsm_ctrl/ekf_ready", 10, Ready_Callback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, State_Callback);
    ros::Subscriber voltage_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("/mavros/battery", 10, Voltage_Callback);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, Pose_Callback);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, Vel_Callback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data", 3, IMU_Callback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
        ("/mavros/rc/in", 10, RC_Callback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/camera/odom/sample", 3, Odom_Callback);
    ros::Subscriber traj_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
        ("/drone_0_planning/pos_cmd", 10, Traj_Callback);
    // SYX FSM TEST
    // We can try new control using this topic 
    ros::Subscriber nmpc_traj_sub = nh.subscribe<traj_utils::Flag>("/position_cmd_nmpc", 10, nmpc_traj_cb);
 
    ros::Subscriber ego_flag_state_sub = nh.subscribe<traj_utils::FlagState>("/ego_planner/flag_state", 10, EGO_flag_state_cb);
    // SYX FSM TEST DONE

    /*----------LXK----------*/
    ros::Subscriber ring_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target_pose", 10, ring_cb);
    ros::Subscriber apriltag_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tf_output", 10, apriltag_cb);
    ros::Subscriber tunnel_sub = nh.subscribe<nav_msgs::Path>("/task2_pose", 10, tunnel_cb);
    ros::Subscriber maze_sub = nh.subscribe<nav_msgs::Path>("/task3_pose", 10, maze_cb);
    /*----------LXK----------*/

    //chx
    ros::Subscriber doublering1_sub = nh.subscribe<geometry_msgs::PoseStamped>("/Lcircle_pos", 10, double_ring1_cb);
    ros::Subscriber doublering2_sub = nh.subscribe<geometry_msgs::PoseStamped>("/Rcircle_pos", 10, double_ring2_cb);
    ros::Publisher appos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/globalappos", 10);

    /*----------YYZ----------*/
    ros::Subscriber planner_msgs_sub = nh.subscribe<super_msgs::Flag>("/super/flag_cmd", 10, PlannerCallback);
    /*----------YYZ----------*/

    /*----------LYX----------*/
    ros::Subscriber planner_state_sub = nh.subscribe<super_msgs::Flag>("/super/flag_state", 10, SUPER_flag_state_cb);
    ros::Publisher planner_cmd_pub = nh.advertise<super_msgs::Flag>("/super/flag_waypoint", 10);
    /*----------LYX----------*/
    

    ros::Publisher task_pub = nh.advertise<std_msgs::Int64>("/perc/task", 10);

    /*--------- Client ---------*/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(RATE); // the setpoint publishing rate must be faster than 2Hz

    Eigen::Vector4d kp, kpi, kpd, kv, kvi, kvd;
    double nmpc_mQ_px, nmpc_mQ_py, nmpc_mQ_pz;
    double nmpc_mQ_vx, nmpc_mQ_vy, nmpc_mQ_vz;
    double nmpc_mQ_q1, nmpc_mQ_q2, nmpc_mQ_q3, nmpc_mQ_q4;
    double nmpc_mR_a_BZ, nmpc_mR_roll, nmpc_mR_pitch, nmpc_mR_yaw;
    double nmpc_hover_thrust;
    // parameters
    nh.param("single_offboard_fsm/kp_x", kp(0), 0.0);
    nh.param("single_offboard_fsm/kp_y", kp(1), 0.0);
    nh.param("single_offboard_fsm/kp_z", kp(2), 0.0);
    nh.param("single_offboard_fsm/kp_yaw", kp(3), 0.0);
    nh.param("single_offboard_fsm/kpi_x", kpi(0), 0.0);
    nh.param("single_offboard_fsm/kpi_y", kpi(1), 0.0);
    nh.param("single_offboard_fsm/kpi_z", kpi(2), 0.0);
    nh.param("single_offboard_fsm/kpi_yaw", kpi(3), 0.0);
    nh.param("single_offboard_fsm/kpd_x", kpd(0), 0.0);
    nh.param("single_offboard_fsm/kpd_y", kpd(1), 0.0);
    nh.param("single_offboard_fsm/kpd_z", kpd(2), 0.0);
    nh.param("single_offboard_fsm/kd_yaw", kpd(3), 0.0);
    nh.param("single_offboard_fsm/kv_x", kv(0), 0.0);
    nh.param("single_offboard_fsm/kv_y", kv(1), 0.0);
    nh.param("single_offboard_fsm/kv_z", kv(2), 0.0);
    nh.param("single_offboard_fsm/kv_yaw", kv(3), 0.0);
    nh.param("single_offboard_fsm/kvi_x", kvi(0), 0.0);
    nh.param("single_offboard_fsm/kvi_y", kvi(1), 0.0);
    nh.param("single_offboard_fsm/kvi_z", kvi(2), 0.0);
    nh.param("single_offboard_fsm/kvi_yaw", kvi(3), 0.0);
    nh.param("single_offboard_fsm/kvd_x", kvd(0), 0.0); 
    nh.param("single_offboard_fsm/kvd_y", kvd(1), 0.0);
    nh.param("single_offboard_fsm/kvd_z", kvd(2), 0.0);
    nh.param("single_offboard_fsm/kvd_yaw", kvd(3), 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_px", nmpc_mQ_px, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_py", nmpc_mQ_py, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_pz", nmpc_mQ_pz, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_vx", nmpc_mQ_vx, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_vy", nmpc_mQ_vy, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_vz", nmpc_mQ_vz, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_q1", nmpc_mQ_q1, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_q2", nmpc_mQ_q2, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_q3", nmpc_mQ_q3, 0.0);
    nh.param("single_offboard_fsm/nmpc_mQ_q4", nmpc_mQ_q4, 0.0);
    nh.param("single_offboard_fsm/nmpc_mR_a_BZ", nmpc_mR_a_BZ, 0.0);
    nh.param("single_offboard_fsm/nmpc_mR_roll", nmpc_mR_roll, 0.0);
    nh.param("single_offboard_fsm/nmpc_mR_pitch", nmpc_mR_pitch, 0.0);
    nh.param("single_offboard_fsm/nmpc_mR_yaw", nmpc_mR_yaw, 0.0);
    nh.param("single_offboard_fsm/nmpc_hover_thrust", nmpc_hover_thrust, 0.196);

    // NMPC Parameter set
    int nmpc_predict_step = 5;
    int nmpc_timestamp = 0;
    float nmpc_sample_time = 0.1;
    int nmpc_test_state=0;
    int nmpc_test_time_cnt=0;
    ros::Time begin_time = ros::Time::now();
    int nmpc_init_flag = 0;
    Eigen::Matrix<float, 10, 1> m_Q_param;
    Eigen::Matrix<float, 4, 1> m_R_param;
    m_Q_param << nmpc_mQ_px, nmpc_mQ_py, nmpc_mQ_pz, nmpc_mQ_vx, nmpc_mQ_vy, nmpc_mQ_vz, nmpc_mQ_q1, nmpc_mQ_q2,
        nmpc_mQ_q3, nmpc_mQ_q4;
    m_R_param << nmpc_mR_a_BZ, nmpc_mR_roll, nmpc_mR_pitch, nmpc_mR_yaw;


    NMPC_Ctrller younger_ctrl(nmpc_predict_step, nmpc_sample_time, INTERV, nmpc_hover_thrust, m_Q_param, m_R_param);

    // NMPC States Init
    Eigen::Matrix<float, 10, 1> nmpc_current_states;   //无人机期望状态（控制目标）   10*1
    Eigen::Matrix<float, 60, 1> nmpc_desired_states;   //无人机期望控制量（控制目标） 10*1
    Eigen::Matrix<float, 4, 1> nmpc_desired_controls;  //最优序列的第一个控制量       4*1

    /*
    状态量说明：
    机体坐标系在世界坐标系下的位置(xyz) 3*1
    机体坐标系在世界坐标系下的速度(xyz) 3*1
    机体坐标系在世界坐标系下的姿态（wxyz）4*1
    */
    nmpc_desired_controls << 0, 0, 0, 0;  // a,omega_roll,omega_pitch,omega_yaw

    int test_yaw = 0;

    Set_AimPos(0.0, 0.0, HEIGHT);

    new std::thread(&UdpListen, 12001);
    // wait for FCU connection
    /*
        while(ros::ok() && !current_state.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }
    */

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "LAND";
    mavros_msgs::SetMode mode_cmd;
    mavros_msgs::CommandBool arm_cmd, disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();
    ros::Time last_goal;
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(aim_pos);
        ros::spinOnce();
        rate.sleep();
    }

    // FLAGCtrller ctrller(RATE, 0.33, kp, kpi, kpd, kv, kvi, kvd);

    /*----------LXK----------*/
    EgoGeneTraj();
    /*----------LXK----------*/

    // SYX FSM TEST
    ros::Time flagFSM = ros::Time::now();
    ros::Time flagReplanFSM = ros::Time::now();
    bool flag_istimer_done = false;
    bool flag_issend_done = false;
    // BOOL FLAG_isreplan = false;
    // SYX FSM TEST DONE


    /*----------YYZ's NMPC 250526----------*/
    float nmpc_Qposx, nmpc_Qposy, nmpc_Qposz;
    float nmpc_Qvelx, nmpc_Qvely, nmpc_Qvelz;
    float nmpc_Rwx, nmpc_Rwy, nmpc_Rwz;
    float nmpc_Qquatx, nmpc_Qquaty, nmpc_Qquatz;
    double nmpc_costRtotalF;

    nh.param("single_offboard_fsm/nmpc_Qposx", nmpc_Qposx, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qposy", nmpc_Qposy, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qposz", nmpc_Qposz, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qvelx", nmpc_Qvelx, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qvely", nmpc_Qvely, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qvelz", nmpc_Qvelz, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Rwx", nmpc_Rwx, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Rwy", nmpc_Rwy, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Rwz", nmpc_Rwz, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qquatx", nmpc_Qquatx, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qquaty", nmpc_Qquaty, 1.0f);
    nh.param("single_offboard_fsm/nmpc_Qquatz", nmpc_Qquatz, 1.0f);
    nh.param("single_offboard_fsm/nmpc_RtotalF", nmpc_costRtotalF, 1.0);

    /*---------- Trajectory generator params ----------*/
    bool use_traj_gen = false;
    std::string traj_type = "circle";
    double traj_cx = 0.0, traj_cy = 0.0, traj_alt = 1.0;
    double traj_param1 = 2.0, traj_param2 = 0.5;

    nh.param("single_offboard_fsm/use_traj_gen", use_traj_gen, false);
    nh.param("single_offboard_fsm/traj_type", traj_type, std::string("circle"));
    nh.param("single_offboard_fsm/traj_cx", traj_cx, 0.0);
    nh.param("single_offboard_fsm/traj_cy", traj_cy, 0.0);
    nh.param("single_offboard_fsm/traj_alt", traj_alt, 1.0);
    nh.param("single_offboard_fsm/traj_param1", traj_param1, 2.0);
    nh.param("single_offboard_fsm/traj_param2", traj_param2, 0.5);

    double traj_hold_time = 3.0;  // seconds to hover before starting motion
    nh.param("single_offboard_fsm/traj_hold_time", traj_hold_time, 3.0);

    std::shared_ptr<traj_gen::Trajectory> internal_traj;
    ros::Time traj_start_time;
    bool traj_active = false;

    /*------------------------------------------------*/

    int nmpc_simple_predict_step = 8;
    double nmpc_simple_sample_time = 0.05;

    std::array<double, 2> _w_limit = {-3.14, 3.14};
    std::array<double, 2> _acc_z_limit = {0.0, 15.0};
    Eigen::Matrix<float, 3, 1> nmpc_costQpos = {nmpc_Qposx, nmpc_Qposy, nmpc_Qposz};
    Eigen::Matrix<float, 3, 1> nmpc_costQvel = {nmpc_Qvelx, nmpc_Qvely, nmpc_Qvelz};
    Eigen::Matrix<float, 3, 1> nmpc_costRw = {nmpc_Rwx, nmpc_Rwy, nmpc_Rwz};
    Eigen::Matrix<float, 3, 1> nmpc_costQquat = {nmpc_Qquatx, nmpc_Qquaty, nmpc_Qquatz};

    NMPC_Ctrller_simple nmpc_ctrl_simple(INTERV, _acc_z_limit, _w_limit, nmpc_simple_predict_step, nmpc_simple_sample_time, 10, 4, nmpc_costQpos, nmpc_costQvel, nmpc_costQquat, nmpc_costRw, nmpc_costRtotalF, nmpc_hover_thrust);

    std::vector<double> current_states;
    std::vector<double> desired_states;

    double w_x_cmd = 0;
    double w_y_cmd = 0;
    double w_z_cmd = 0;





    static int prev_cmd = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        // Reset trajectory generator when switching into cmd=5
        if (cmd == 5 && prev_cmd != 5)
        {
            traj_active = false;
        }
        prev_cmd = cmd;

        // /*----------LXK----------*/
        // //for ego
        if (flag_state.now_id < Ego_traj_size-1)
        {
            task = state2task[flag_state.now_id];
        }
        // //for super
        // if (flag_super_state.now_id < Ego_traj_size-1)
        // {
        //     task = state2task[flag_super_state.now_id];
        // }
        
        // PrintInfo(task_print_count, 25, "task: ", task);


        if (task < 6)
        {
            //for ego
            perc_mode = Ego_traj[flag_state.now_id].perc_mode;
            //for super
            // perc_mode = Ego_traj[flag_super_state.now_id].perc_mode;
        }
        else
        {
            perc_mode = 6;
        }
        // PrintInfo(perc_mode_print_count, 25, "perc mode: ", perc_mode);
        // // PrintInfo(perc_mode_print_count, 25, "cmd: ", cmd);
        if(flag_state.now_id==Ego_traj_size-1){
          
                if(flag_state.touch_goal == true){
                    task1 = 6;
                    cout << "task" <<task1 << endl;
                }
                
            }
        
        //perc_mode = 7;
        perc_mode_msg.data = perc_mode;
        PrintInfo(perc_mode_print_count, 25, "perc mode: ", perc_mode);
        perc_mode_pub.publish(perc_mode_msg);

        ap_gl_msg.pose.position.x = ap_pose(0);
        ap_gl_msg.pose.position.y = ap_pose(1);
        ap_gl_msg.pose.position.z = ap_pose(2);
        ap_pub.publish(ap_gl_msg);
        /*----------LXK----------*/

        /*--------- 解锁 ---------*/
        ROS_INFO_THROTTLE(1.0, "CMD=%d",cmd);

        if (cmd == 1)
        {
            // printf("CMD = 1\n");
            ROS_INFO_THROTTLE(1.0, "CMD=1");
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                ROS_INFO_THROTTLE(1.0, "if 1");
                // printf("%d\n",current_state.mode);
                // cout << current_state.mode << endl
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            target_att.header.frame_id = std::string("FCU");
            target_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

            target_att.body_rate.x = 0.0;
            target_att.body_rate.y = 0.0;
            target_att.body_rate.z = 0.0;

            target_att.thrust = 0.02;
            local_attitude_pub.publish(target_att);
        }

        // /*--------- 位置悬停 ---------*/
        if (cmd == 2)
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            Set_AimPos(0.0, 0.0, 1.0);
            local_pos_pub.publish(aim_pos);
        }

        // /*--------- NMPC悬停 (使用 nmpc_ctrl_simple，四元数误差计算正确) ---------*/
        if (cmd == 3)
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            // NMPC hover using nmpc_ctrl_simple (correct quaternion error)
            current_states.clear();
            desired_states.clear();

            current_states.push_back(pos_fcu.x());
            current_states.push_back(pos_fcu.y());
            current_states.push_back(pos_fcu.z());
            current_states.push_back(vel_fcu.x());
            current_states.push_back(vel_fcu.y());
            current_states.push_back(vel_fcu.z());
            current_states.push_back(quat_fcu.w());
            current_states.push_back(quat_fcu.x());
            current_states.push_back(quat_fcu.y());
            current_states.push_back(quat_fcu.z());


            // desired states: hover at (0, 0, 0.2), level attitude
            for (int i = 0; i < nmpc_simple_predict_step + 1; i++)
            {

                desired_states.push_back(0.0);   // px
                desired_states.push_back(0.0);   // py
                desired_states.push_back(1.0);   // pz
                desired_states.push_back(0.0);   // vx
                desired_states.push_back(0.0);   // vy
                desired_states.push_back(0.0);   // vz
                desired_states.push_back(1.0);   // qw
                desired_states.push_back(0.0);   // qx
                desired_states.push_back(0.0);   // qy
                desired_states.push_back(0.0);   // qz
            }

            // desired controls: zero angular velocity, gravity compensation
            for (int i = 0; i < nmpc_simple_predict_step; i++)
            {
                desired_states.push_back(0.0);      // wx_ref
                desired_states.push_back(0.0);      // wy_ref
                desired_states.push_back(0.0);      // wz_ref
                desired_states.push_back(9.8015);   // acc_z_ref = gravity
            }

            nmpc_ctrl_simple.optimal_solution(current_states, desired_states);

            double acc_z_command = nmpc_ctrl_simple.getAcc_zCommand();
            Eigen::Vector3d w_command = nmpc_ctrl_simple.getwCommand();

            // DEBUG: compare with cmd==5
            ROS_INFO_THROTTLE(0.5, "CMD3 NMPC: pos_fcu=(%.2f,%.2f,%.2f) quat=(%.2f,%.2f,%.2f,%.2f) "
                              "cmd=(wx=%.3f,wy=%.3f,wz=%.3f,thrust=%.3f)",
                              pos_fcu.x(), pos_fcu.y(), pos_fcu.z(),
                              quat_fcu.w(), quat_fcu.x(), quat_fcu.y(), quat_fcu.z(),
                              w_command.x(), w_command.y(), w_command.z(), acc_z_command);

            target_att.header.frame_id = std::string("FCU");
            target_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
            target_att.body_rate.x = w_command.x();
            target_att.body_rate.y = w_command.y();
            target_att.body_rate.z = w_command.z();
            target_att.thrust = acc_z_command;
            local_attitude_pub.publish(target_att);

            std::cout << "thrust is " << acc_z_command << std::endl;
        }
        /*--------- 降落 ---------*/
        if (cmd == 4)
        {
            if (!is_land)
            {
                Set_AimPos(pos_fcu[0], pos_fcu[1], 0.005);
                local_pos_pub.publish(aim_pos);
                if (abs(pos_fcu[2] - 0.05) < 0.05)
                {
                    is_land = true;
                }
            }
            else
            {
                if (current_state.mode == "OFFBOARD")
                {
                    // mode_cmd.request.custom_mode = "Hold";
                    // set_mode_client.call(mode_cmd);
                }
                else
                {
                    if (current_state.armed)
                    {
                        arm_cmd.request.value = false;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_WARN("Mode Disarm!");
                        }
                    }
                }
            }
        }

        if (cmd == 5) //简单测试一下发点
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            ros::Time start_time = ros::Time::now();
            current_states.clear();
            desired_states.clear();

            current_states.push_back(pos_fcu.x());
            current_states.push_back(pos_fcu.y());
            current_states.push_back(pos_fcu.z());
            current_states.push_back(vel_fcu.x());
            current_states.push_back(vel_fcu.y());
            current_states.push_back(vel_fcu.z());
            current_states.push_back(quat_fcu.w());
            current_states.push_back(quat_fcu.x());
            current_states.push_back(quat_fcu.y());
            current_states.push_back(quat_fcu.z());

            if(is_get_planner_msgs == false)
            {
                /* Internal trajectory generator */
                if (use_traj_gen)
                {
                    // Lazy init trajectory on first call
                    if (!traj_active)
                    {
                        if (traj_type == "waypoint")
                        {
                            // Read waypoints from ROS param server (defined in launch file)
                            XmlRpc::XmlRpcValue wp_list;
                            if (nh.getParam("single_offboard_fsm/traj_waypoints", wp_list) &&
                                wp_list.getType() == XmlRpc::XmlRpcValue::TypeArray && wp_list.size() >= 2)
                            {
                                auto wp_traj = std::make_shared<traj_gen::WaypointTrajectory>(
                                    traj_alt, traj_param2, true);
                                for (int k = 0; k < wp_list.size(); k++)
                                {
                                    double x = static_cast<double>(wp_list[k][0]);
                                    double y = static_cast<double>(wp_list[k][1]);
                                    double yaw = (wp_list[k].size() > 2)
                                        ? static_cast<double>(wp_list[k][2]) : 0.0;
                                    wp_traj->addWaypoint(x, y, yaw);
                                }
                                internal_traj = wp_traj;
                                ROS_WARN("Trajectory [waypoint] loaded: %d points, alt=%.2f speed=%.2f",
                                         wp_list.size(), traj_alt, traj_param2);
                            }
                            else
                            {
                                ROS_ERROR("traj_type=waypoint but traj_waypoints not found or <2 points!");
                                internal_traj = traj_gen::createTrajectory(
                                    "circle", traj_cx, traj_cy, traj_alt, traj_param1, traj_param2);
                            }
                        }
                        else
                        {
                            try
                            {
                                internal_traj = traj_gen::createTrajectory(
                                    traj_type, traj_cx, traj_cy, traj_alt, traj_param1, traj_param2);
                            }
                            catch (const std::exception& e)
                            {
                                ROS_ERROR("Failed to create trajectory [%s]: %s. Falling back to circle.",
                                          traj_type.c_str(), e.what());
                                internal_traj = traj_gen::createTrajectory(
                                    "circle", traj_cx, traj_cy, traj_alt, traj_param1, traj_param2);
                            }
                            ROS_WARN("Trajectory [%s] started! alt=%.2f param1=%.2f param2=%.2f",
                                     internal_traj->name().c_str(), traj_alt, traj_param1, traj_param2);
                        }
                        traj_start_time = ros::Time::now();
                        traj_active = true;
                        ROS_WARN("Trajectory starts in %.1fs — holding at (0.00, 0.00, %.2f)",
                                 traj_hold_time, traj_alt);
                    }

                    double t_now = (ros::Time::now() - traj_start_time).toSec();

                    for (int i = 0; i < nmpc_simple_predict_step + 1; i++)
                    {
                        double t_sample = t_now + i * nmpc_simple_sample_time;

                        traj_gen::TrajPoint pt;
                        if (t_sample < traj_hold_time)
                        {
                            // Hold phase: hover at fixed world position, NOT feedback
                            pt.pos = Eigen::Vector3d(0.0, 0.0, traj_alt);
                            pt.vel = Eigen::Vector3d::Zero();
                            pt.yaw = 0.0;
                        }
                        else
                        {
                            // Motion phase: sample trajectory with time offset
                            pt = internal_traj->sample(t_sample - traj_hold_time);
                        }

                        Eigen::Quaterniond quat_yaw_traj = EulerToQuat(0.0, 0.0, pt.yaw);

                        desired_states.push_back(pt.pos.x());
                        desired_states.push_back(pt.pos.y());
                        desired_states.push_back(pt.pos.z());
                        desired_states.push_back(pt.vel.x());
                        desired_states.push_back(pt.vel.y());
                        desired_states.push_back(pt.vel.z());
                        desired_states.push_back(quat_yaw_traj.w());
                        desired_states.push_back(quat_yaw_traj.x());
                        desired_states.push_back(quat_yaw_traj.y());
                        desired_states.push_back(quat_yaw_traj.z());

                        if (i == 0)
                        {
                            const char* phase = (t_now < traj_hold_time) ? "HOLD" : "MOVE";
                            ROS_INFO_THROTTLE(1.0, "Traj gen [%s-%s]: t=%.2f pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f)",
                                              traj_type.c_str(), phase, t_now,
                                              pt.pos.x(), pt.pos.y(), pt.pos.z(),
                                              pt.vel.x(), pt.vel.y());
                        }
                    }

                    // Publish prediction horizon as Path for RViz / rosbag
                    {
                        nav_msgs::Path traj_path_msg;
                        traj_path_msg.header.frame_id = "map";
                        traj_path_msg.header.stamp = ros::Time::now();
                        for (int i = 0; i < nmpc_simple_predict_step + 1; i++)
                        {
                            double t_sample = t_now + i * nmpc_simple_sample_time;
                            traj_gen::TrajPoint pt;
                            if (t_sample < traj_hold_time)
                            {
                                // Hold phase: hover at fixed world position, NOT feedback
                                pt.pos = Eigen::Vector3d(0.0, 0.0, traj_alt);
                                pt.vel = Eigen::Vector3d::Zero();
                                pt.yaw = 0.0;
                            }
                            else
                            {
                                pt = internal_traj->sample(t_sample - traj_hold_time);
                            }
                            geometry_msgs::PoseStamped pose;
                            pose.header = traj_path_msg.header;
                            pose.pose.position.x = pt.pos.x();
                            pose.pose.position.y = pt.pos.y();
                            pose.pose.position.z = pt.pos.z();
                            Eigen::Quaterniond q = EulerToQuat(0.0, 0.0, pt.yaw);
                            pose.pose.orientation.w = q.w();
                            pose.pose.orientation.x = q.x();
                            pose.pose.orientation.y = q.y();
                            pose.pose.orientation.z = q.z();
                            traj_path_msg.poses.push_back(pose);
                        }
                        traj_path_pub.publish(traj_path_msg);
                        nmpc_posref_pub.publish(traj_path_msg.poses.front());
                    }
                }
                else
                {
                    /* Original fallback: hover */
                    for(int i = 0; i < nmpc_simple_predict_step + 1; i++)
                    {
                        desired_states.push_back(0.00);
                        desired_states.push_back(0.00);
                        desired_states.push_back(0.5);
                        desired_states.push_back(0.0);
                        desired_states.push_back(0.0);
                        desired_states.push_back(0.0);
                        desired_states.push_back(1.0);
                        desired_states.push_back(0.0);
                        desired_states.push_back(0.0);
                        desired_states.push_back(0.0);
                        ROS_INFO("get nmpc traj  pos_des[i] = %f", 0.0);
                    }
                }
            }
            else
            {
                for(int i = 0; i < nmpc_simple_predict_step + 1; i++)
                {
                    // nmpc_traj_cb only fills indices 0-5; clamp to avoid reading uninit data
                    int pt_idx = std::min(i, 5);
                    desired_states.push_back(nmpc_traj_pt[pt_idx].pos.x());
                    desired_states.push_back(nmpc_traj_pt[pt_idx].pos.y());
                    desired_states.push_back(nmpc_traj_pt[pt_idx].pos.z());
                    desired_states.push_back(nmpc_traj_pt[pt_idx].vel.x());
                    desired_states.push_back(nmpc_traj_pt[pt_idx].vel.y());
                    desired_states.push_back(nmpc_traj_pt[pt_idx].vel.z());
                    desired_states.push_back(1.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    ROS_INFO("get nmpc traj  pos_des[i] = %f", nmpc_traj_pt[pt_idx].pos.x());
                }
            }

            
            for(int i = 0; i < nmpc_simple_predict_step; i++)
            {
                desired_states.push_back(0.0); 
                desired_states.push_back(0.0);
                desired_states.push_back(0.0);
                desired_states.push_back(9.8015);
            }

            nmpc_ctrl_simple.optimal_solution(current_states, desired_states);

            double acc_z_command = nmpc_ctrl_simple.getAcc_zCommand();
            Eigen::Vector3d w_command = nmpc_ctrl_simple.getwCommand();

            // DEBUG: compare with cmd==3
            ROS_INFO_THROTTLE(0.5, "CMD5 NMPC: pos_fcu=(%.2f,%.2f,%.2f) quat=(%.2f,%.2f,%.2f,%.2f) "
                              "cmd=(wx=%.3f,wy=%.3f,wz=%.3f,thrust=%.3f) "
                              "des_p0=(%.2f,%.2f,%.2f)",
                              pos_fcu.x(), pos_fcu.y(), pos_fcu.z(),
                              quat_fcu.w(), quat_fcu.x(), quat_fcu.y(), quat_fcu.z(),
                              w_command.x(), w_command.y(), w_command.z(), acc_z_command,
                              desired_states[0], desired_states[1], desired_states[2]);

            target_att.header.frame_id = std::string("FCU");
            target_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

            target_att.body_rate.x = w_command.x();
            target_att.body_rate.y = w_command.y();
            target_att.body_rate.z = w_command.z();

            target_att.thrust = acc_z_command;
            local_attitude_pub.publish(target_att);

            // 发布参考轨迹点 & 实际位置，方便 RViz / rqt_plot 监视
            {
                geometry_msgs::PoseStamped nmpc_posref_msg;
                nmpc_posref_msg.header.stamp = ros::Time::now();
                nmpc_posref_msg.header.frame_id = "world";
                nmpc_posref_msg.pose.position.x = desired_states[0];
                nmpc_posref_msg.pose.position.y = desired_states[1];
                nmpc_posref_msg.pose.position.z = desired_states[2];
                nmpc_posref_msg.pose.orientation.w = 1.0;
                nmpc_posref_pub.publish(nmpc_posref_msg);

                geometry_msgs::PoseStamped nmpc_posfdb_msg;
                nmpc_posfdb_msg.header.stamp = ros::Time::now();
                nmpc_posfdb_msg.header.frame_id = "world";
                nmpc_posfdb_msg.pose.position.x = pos_fcu(0);
                nmpc_posfdb_msg.pose.position.y = pos_fcu(1);
                nmpc_posfdb_msg.pose.position.z = pos_fcu(2);
                nmpc_posfdb_msg.pose.orientation.w = 1.0;
                nmpc_posfdb_pub.publish(nmpc_posfdb_msg);
            }

            ros::Time end_time = ros::Time::now();
            ros::Duration elapsed_time = end_time - start_time;
            if(elapsed_time.toSec() * 1000 > 12)
            {ROS_WARN("NMPC used time: %.1f ms", elapsed_time.toSec() * 1000);}

            fsm_ctrl::nmpc_state nmpc_state_msg;

            for (int i = 0; i < 9; i++) {
                nmpc_state_msg.pos_ref[i].x = desired_states[i * 10];     // x坐标赋值
                nmpc_state_msg.pos_ref[i].y = desired_states[i * 10 + 1];     // y坐标赋值
                nmpc_state_msg.pos_ref[i].z = desired_states[i * 10 + 2];     // z坐标赋值
                nmpc_state_msg.vel_ref[i].x = desired_states[i * 10 + 3];     // x速度赋值
                nmpc_state_msg.vel_ref[i].y = desired_states[i * 10 + 4];     // y速度赋值
                nmpc_state_msg.vel_ref[i].z = desired_states[i * 10 + 5];     // z速度赋值
            }

            nmpc_state_msg.pos_fdb.x = pos_fcu(0);
            nmpc_state_msg.pos_fdb.y = pos_fcu(1);
            nmpc_state_msg.pos_fdb.z = pos_fcu(2);

            nmpc_state_msg.vel_fdb.x = vel_fcu(0);
            nmpc_state_msg.vel_fdb.y = vel_fcu(1);
            nmpc_state_msg.vel_fdb.z = vel_fcu(2);

            nmpc_state_msg.attitude_fdb.x = quat_fcu.x();
            nmpc_state_msg.attitude_fdb.y = quat_fcu.y();
            nmpc_state_msg.attitude_fdb.z = quat_fcu.z();
            nmpc_state_msg.attitude_fdb.w = quat_fcu.w();

            nmpc_state_msg.target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                                  mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
            nmpc_state_msg.target.body_rate.x = w_command.x();
            nmpc_state_msg.target.body_rate.y = w_command.y();
            nmpc_state_msg.target.body_rate.z = w_command.z();
            nmpc_state_msg.target.thrust = acc_z_command;

            nmpc_state_pub.publish(nmpc_state_msg);
        }

        
        if (cmd == 8)
        {
            if(need_GeneTraj){
                EgoGeneTraj();
                if (Ego_traj_count<Ego_traj_size){
                    EGO_flag_aimpos(Ego_traj[Ego_traj_count]);
                    ROS_INFO("Ego Trajectory Num %d", Ego_traj_count);
                    ROS_INFO("Flag %d: x: %f, y: %f, z: %f", Ego_traj_count, Ego_traj[Ego_traj_count].x, Ego_traj[Ego_traj_count].y, Ego_traj[Ego_traj_count].z);
                    // planner_cmd_pub.publish(flag_super_msg);
                    ego_flag_pub.publish(flag_traj_msg);
                }           
                Ego_traj_count++;
                if (Ego_traj_count == Ego_traj_size)
                {
                    need_GeneTraj = false;
                    Ego_traj_count = 0;
                }
            }

            quat_yaw = EulerToQuat(0, 0, YawSmooth(yaw_now, Ego_traj[flag_state.now_id].yaw));
            ROS_INFO("yaw is %f", yaw_now);
            
            geometry_msgs::PoseStamped nmpc_posfdb_msg;
            nmpc_posfdb_msg.pose.position.x = pos_fcu(0);
            nmpc_posfdb_msg.pose.position.y = pos_fcu(1);
            nmpc_posfdb_msg.pose.position.z = pos_fcu(2);
            nmpc_posfdb_pub.publish(nmpc_posfdb_msg);
            std::cout<<"position[0]: "<<pos_fcu(0)<<std::endl;

            geometry_msgs::PoseStamped nmpc_posref_msg;
            nmpc_posref_msg.pose.position.x = nmpc_traj_pt[0].pos(0);
            nmpc_posref_msg.pose.position.y = nmpc_traj_pt[0].pos(1);
            nmpc_posref_msg.pose.position.z = nmpc_traj_pt[0].pos(2);
            nmpc_posref_pub.publish(nmpc_posref_msg);

            // NMPC data update
            nmpc_current_states << pos_fcu(0),pos_fcu(1),pos_fcu(2),vel_fcu(0),vel_fcu(1),vel_fcu(2),quat_fcu.w(),quat_fcu.x(),quat_fcu.y(),quat_fcu.z();
            nmpc_desired_states << 
            nmpc_traj_pt[0].pos(0), nmpc_traj_pt[0].pos(1), nmpc_traj_pt[0].pos(2),
            nmpc_traj_pt[0].vel(0), nmpc_traj_pt[0].vel(1), nmpc_traj_pt[0].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[1].pos(0), nmpc_traj_pt[1].pos(1), nmpc_traj_pt[1].pos(2), 
            nmpc_traj_pt[1].vel(0), nmpc_traj_pt[1].vel(1), nmpc_traj_pt[1].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[2].pos(0), nmpc_traj_pt[2].pos(1), nmpc_traj_pt[2].pos(2), 
            nmpc_traj_pt[2].vel(0), nmpc_traj_pt[2].vel(1), nmpc_traj_pt[2].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[3].pos(0), nmpc_traj_pt[3].pos(1), nmpc_traj_pt[3].pos(2), 
            nmpc_traj_pt[3].vel(0), nmpc_traj_pt[3].vel(1), nmpc_traj_pt[3].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[4].pos(0), nmpc_traj_pt[4].pos(1), nmpc_traj_pt[4].pos(2), 
            nmpc_traj_pt[4].vel(0), nmpc_traj_pt[4].vel(1), nmpc_traj_pt[4].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[5].pos(0), nmpc_traj_pt[5].pos(1), nmpc_traj_pt[5].pos(2), 
            nmpc_traj_pt[5].vel(0), nmpc_traj_pt[5].vel(1), nmpc_traj_pt[5].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z();  //期望的位置
            nmpc_desired_controls << 9.8, 0, 0, 0;                  //期望的控制量
            // NMPC solution
            younger_ctrl.optimal_solution(nmpc_current_states, nmpc_desired_states, nmpc_desired_controls);

            if(younger_ctrl.Acc2Trust())
            {
                mavros_msgs::AttitudeTarget msg;
                msg.header.frame_id = std::string("NMPC");
                msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
                msg.thrust = younger_ctrl.get_control_command()(0);
                msg.body_rate.x = younger_ctrl.get_control_command()(1);
                msg.body_rate.y = younger_ctrl.get_control_command()(2);
                msg.body_rate.z = younger_ctrl.get_control_command()(3);

                std::cout<<"thrust is "<<younger_ctrl.get_control_command()(0)<<std::endl;
                local_attitude_pub.publish(msg);
            }
        }

        if (cmd == 6){       //jieyixia super
            // std::cout<<"cmd=6"<<std::endl;

            // Offboard & Arm (same as cmd==5)
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            if(need_GeneTraj){
                EgoGeneTraj();
                if (Ego_traj_count<Ego_traj_size){
                    EGO_flag_aimpos(Ego_traj[Ego_traj_count]);//给定点传到全局变量
                    ROS_INFO("Ego Trajectory Num %d", Ego_traj_count);
                    ROS_INFO("Flag %d: x: %f, y: %f, z: %f", Ego_traj_count, Ego_traj[Ego_traj_count].x, Ego_traj[Ego_traj_count].y, Ego_traj[Ego_traj_count].z);
                    //planner_cmd_pub.publish(flag_super_msg);
                    // ego_flag_pub.publish(flag_traj_msg);
                }           
                Ego_traj_count++;
                    if (Ego_traj_count == Ego_traj_size)
                    {
                        need_GeneTraj = false;
                        Ego_traj_count = 0;
                    }
            }

            quat_yaw = EulerToQuat(0, 0, YawSmooth(yaw_now, Ego_traj[flag_state.now_id].yaw));

            ros::Time start_time = ros::Time::now();
            current_states.clear();
            desired_states.clear();

            current_states.push_back(pos_fcu.x());
            current_states.push_back(pos_fcu.y());
            current_states.push_back(pos_fcu.z());
            current_states.push_back(vel_fcu.x());
            current_states.push_back(vel_fcu.y());
            current_states.push_back(vel_fcu.z());
            current_states.push_back(quat_fcu.w());
            current_states.push_back(quat_fcu.x());
            current_states.push_back(quat_fcu.y());
            current_states.push_back(quat_fcu.z());

            if(is_get_planner_msgs == false)
            {
                for(int i = 0; i < nmpc_simple_predict_step + 1; i++)
                {
                    desired_states.push_back(0.00); 
                    desired_states.push_back(0.00);
                    desired_states.push_back(1.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(1.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    // ROS_INFO("get nmpc traj  pos_des[i] = %f", 0.0);
                }
            }
            else
            {
                for(int i = 0; i < nmpc_simple_predict_step + 1; i++)
                {
                    // for super
                    desired_states.push_back(nmpc_pos_des[i].x());
                    desired_states.push_back(nmpc_pos_des[i].y());
                    desired_states.push_back(nmpc_pos_des[i].z());
                    desired_states.push_back(nmpc_vel_des[i].x());
                    desired_states.push_back(nmpc_vel_des[i].y());
                    desired_states.push_back(nmpc_vel_des[i].z());
                    desired_states.push_back(1.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);
                    desired_states.push_back(0.0);

                    //for egov2
                    // nmpc_traj_cb only fills indices 0-5; clamp to avoid reading uninit data
                    // int pt_idx = std::min(i, 5);
                    // desired_states.push_back(nmpc_traj_pt[pt_idx].pos.x());
                    // desired_states.push_back(nmpc_traj_pt[pt_idx].pos.y());
                    // desired_states.push_back(nmpc_traj_pt[pt_idx].pos.z());
                    // desired_states.push_back(nmpc_traj_pt[pt_idx].vel.x());
                    // desired_states.push_back(nmpc_traj_pt[pt_idx].vel.y());
                    // desired_states.push_back(nmpc_traj_pt[pt_idx].vel.z());
                    // desired_states.push_back(quat_yaw.w());
                    // desired_states.push_back(quat_yaw.x());
                    // desired_states.push_back(quat_yaw.y());
                    // desired_states.push_back(quat_yaw.z());
                    // ROS_INFO("get nmpc traj  yaw = %f", yaw_now);

                }
            }

             
            for(int i = 0; i < nmpc_simple_predict_step; i++)
            {
                desired_states.push_back(0.0); 
                desired_states.push_back(0.0);
                desired_states.push_back(0.0);
                desired_states.push_back(9.8015);
            }

            nmpc_ctrl_simple.optimal_solution(current_states, desired_states);

            double acc_z_command = nmpc_ctrl_simple.getAcc_zCommand();
            Eigen::Vector3d w_command = nmpc_ctrl_simple.getwCommand();

            target_att.header.frame_id = std::string("FCU");
            target_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

            target_att.body_rate.x = w_command.x();
            target_att.body_rate.y = w_command.y();
            target_att.body_rate.z = w_command.z();

            target_att.thrust = acc_z_command;
            local_attitude_pub.publish(target_att);

            ros::Time end_time = ros::Time::now();
            ros::Duration elapsed_time = end_time - start_time;
            if(elapsed_time.toSec() * 1000 > 1)
            {ROS_WARN("NMPC used time: %.1f ms", elapsed_time.toSec() * 1000);}

            fsm_ctrl::nmpc_state nmpc_state_msg;

            for (int i = 0; i < 9; i++) {
                nmpc_state_msg.pos_ref[i].x = desired_states[i * 10];     // x坐标赋值
                nmpc_state_msg.pos_ref[i].y = desired_states[i * 10 + 1];     // y坐标赋值
                nmpc_state_msg.pos_ref[i].z = desired_states[i * 10 + 2];     // z坐标赋值
                nmpc_state_msg.vel_ref[i].x = desired_states[i * 10 + 3];     // x速度赋值
                nmpc_state_msg.vel_ref[i].y = desired_states[i * 10 + 4];     // y速度赋值
                nmpc_state_msg.vel_ref[i].z = desired_states[i * 10 + 5];     // z速度赋值
            }

            nmpc_state_msg.pos_fdb.x = current_states[0];
            nmpc_state_msg.pos_fdb.y = current_states[1];
            nmpc_state_msg.pos_fdb.z = current_states[2];

            nmpc_state_msg.vel_fdb.x = current_states[3];
            nmpc_state_msg.vel_fdb.y = current_states[4];
            nmpc_state_msg.vel_fdb.z = current_states[5];

            nmpc_state_msg.attitude_fdb.x = current_states[6];
            nmpc_state_msg.attitude_fdb.y = current_states[7];
            nmpc_state_msg.attitude_fdb.z = current_states[8];
            nmpc_state_msg.attitude_fdb.w = current_states[9];

            nmpc_state_msg.target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                                  mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
            nmpc_state_msg.target.body_rate.x = w_command.x();
            nmpc_state_msg.target.body_rate.y = w_command.y();
            nmpc_state_msg.target.body_rate.z = w_command.z();
            nmpc_state_msg.target.thrust = acc_z_command;

            nmpc_state_pub.publish(nmpc_state_msg);

        }

        if (cmd ==7)
        {
            Ego_traj.clear();
            if(need_GeneTraj){
                EgoGeneTraj();
                if (Ego_traj_count<Ego_traj_size){
                    EGO_flag_aimpos(Ego_traj[Ego_traj_count]);
                    ROS_INFO("Ego Trajectory Num %d", Ego_traj_count);
                    ROS_INFO("Flag %d: x: %f, y: %f, z: %f", Ego_traj_count, Ego_traj[Ego_traj_count].x, Ego_traj[Ego_traj_count].y, Ego_traj[Ego_traj_count].z);
                    // planner_cmd_pub.publish(flag_super_msg);
                    ego_flag_pub.publish(flag_traj_msg);
                }           
                Ego_traj_count++;
                if (Ego_traj_count == Ego_traj_size)
                {
                    need_GeneTraj = false;
                    Ego_traj_count = flag_state.now_id;
                }
            }
            dynringpost_pose(0)=ring2_pose(0)-1;
            dynringpost_pose(1)=ring2_pose(1);
            
        
            if( task1 == 6 ){
              
                if (is_arrive_ring2 == false && is_crossed_ring2 == false)
                {
                    double delta_pos_y = 0.4 * Clamp_Single(ring2_pose(1) - pos_fcu[1], 0.1);
                    for (int i=0;i<6;i++)
                    {
                        nmpc_traj_pt[i].pos(0) = ring2_pose(0) + 1 - 0.01*ring2_approach_count;    //
                        nmpc_traj_pt[i].pos(1) = ring2_pose(1) + i*delta_pos_y/3;
                        nmpc_traj_pt[i].pos(2) = 1.65;
                        nmpc_traj_pt[i].vel(0) = 0.0;
                        nmpc_traj_pt[i].vel(1) = 0.0;
                        nmpc_traj_pt[i].vel(2) = 0.0;
                    }
                    cout << "Flying to Ring2!  [Delta_x, Delta_y]: [" << ring2_pose(0)-pos_fcu[0] << ", " << ring2_pose(1)-pos_fcu[1] << "]" << endl;
                    ring2_approach_count++;
                    if (ring2_approach_count == 100)
                    {
                        is_arrive_ring2 = true;
                    }
                }
                else if (is_arrive_ring2 == true && is_crossed_ring2 == false)
                {
                    for (int i=0;i<6;i++)
                    {
                        nmpc_traj_pt[i].pos(0) = ring2_pose(0) - 0.01*ring2_cross_count;
                        nmpc_traj_pt[i].pos(1) = ring2_pose(1);
                        nmpc_traj_pt[i].pos(2) = 1.65;
                        nmpc_traj_pt[i].vel(0) = 0.0;
                        nmpc_traj_pt[i].vel(1) = 0.0;
                        nmpc_traj_pt[i].vel(2) = 0.0;
                    }
                    cout << "Crossing Ring2!  " << ring2_cross_count << endl;
                    ring2_cross_count++;
                    if (ring2_cross_count == 100)
                    {
                        is_crossed_ring2 = true;
                    }
                }
                else if (is_arrive_ring2 == true && is_crossed_ring2 == true)
                if (is_arrive_ring2 == true && is_crossed_ring2 == true)
                {
                    
                    if (count1<150)
                    {   
                        double delta_pos_x = (appre_pose(0)-dynringpost_pose(0))/150;
                        double delta_pos_y = (appre_pose(1)-dynringpost_pose(1))/150;
                        double delta_pos_z = 0.15 * Clamp_Single(appre_pose(2) - pos_fcu[2], 0.1);
                        cout << "flying to ring post point!" << endl;
                        count1++;
                        for (int i=0;i<6;i++)
                        {
                            nmpc_traj_pt[i].pos(0) = dynringpost_pose(0) + delta_pos_x*count1;
                            nmpc_traj_pt[i].pos(1) = dynringpost_pose(1) + delta_pos_y*count1;
                            nmpc_traj_pt[i].pos(2) = 1.65;
                            nmpc_traj_pt[i].vel(0) = 0.0;
                            nmpc_traj_pt[i].vel(1) = 0.0;
                            nmpc_traj_pt[i].vel(2) = 0.0;
                            cout << "nmpc_traj_pt[i].pos(0) is :" << nmpc_traj_pt[i].pos(0) <<  endl;
                            cout << "nmpc_traj_pt[i].pos(1) is :" << nmpc_traj_pt[i].pos(1) <<  endl;
                            cout << "nmpc_traj_pt[i].pos(2) is :" << nmpc_traj_pt[i].pos(2) <<  endl;
                        }  
                    }
                    else
                    {
                        is_arrive_r2_postpoint = true;
                    }
                    // if (is_send_r2_postpoint == false)
                    // {
                    //     TypePoint ap_target;
                    //     ap_target.id = Ego_traj_size;
                    //     ap_target.mode = 1;
                    //     ap_target.is_map = 0;
                    //     ap_target.x = 13.249;
                    //     ap_target.y = 0.128;
                    //     ap_target.z = 1.28;
                    //     // ap_target.id = 2;
                    //     // ap_target.mode = 2;
                    //     // ap_target.is_map = 0;
                    //     // ap_target.x = ring2_pose(0)+1;
                    //     // ap_target.y = ring2_pose(1);
                    //     // ap_target.z = 0.5;
                    //     EGO_flag_aimpos(ap_target);
                    //     ROS_INFO("Flying to Ring2 Postpoint!");
                    //     // ROS_INFO("Publishing point %d", Ego_traj_size);
                    //     ROS_INFO("finish");
                    //     // ROS_INFO("Flag 2: x: %f, y: %f, z: %f",  ap_target.x, ap_target.y, ap_target.z);
                    //     ego_flag_pub.publish(flag_traj_msg);
                    //     is_send_r2_postpoint= true;
                    // }
                    // else
                    // {
                    //     PrintInfo(r2_print_count, 20, "Flying to Ring2 Postpoint!!!");
                    // }
                    // if (flag_state.touch_goal == true)
                    // {
                    //     if (r2_postpoint_lock < 10)
                    //     {
                    //         r2_postpoint_lock++;
                    //     }
                    //     else
                    //     {
                    //         is_arrive_r2_postpoint = true;
                    //         cout << "Arrive Ring2 Postpoint!" << endl;
                    //         cout << "Arrive Ring2 Postpoint!" << endl;
                    //         cout << "Arrive Ring2 Postpoint!" << endl;
                    //     }
                    // }
                }
            }

            if(is_arrive_r2_postpoint == true){
                if (is_found_ap == false && is_arrive_ap_prepoint == false)
                    {
                    PrintInfo(ap_print_count, 50, "waitingfor Apriltag!"); 
                    // if (ap_search_lock == false)
                    // {
                    //     if (flag_state.touch_goal == true)
                    //     {
                    //         search_orient++;
                    //         TypePoint search_point;
                    //         search_point.id = Ego_traj_size + search_orient;
                    //         search_point.mode = 2;
                    //         search_point.is_map = 0;
                    //         search_point.x = ap_pose(0);
                    //         search_point.y = ap_pose(1) + pow(-1, search_orient%2)*1;
                    //         search_point.z = 1.28;
                    //         EGO_flag_aimpos(search_point);
                    //         ROS_INFO("Publishing point %d", Ego_traj_size+search_orient);
                    //         ROS_INFO("Flag %d: x: %f, y: %f, z: %f", Ego_traj_size+search_orient, search_point.x, search_point.y, search_point.z);
                    //         ROS_INFO("Searching index %d", search_orient);
                    //         ego_flag_pub.publish(flag_traj_msg);
                    //         ap_search_lock = true;
                    //     }
                    // }
                    // else
                    // {
                    //     if (ap_search_lock_count < 10)
                    //     {
                    //         ap_search_lock_count++;
                    //     }
                    //     else
                    //     {
                    //         ap_search_lock_count = 0;
                    //         ap_search_lock = false;
                    //     }
                    // }   
                    }            
                if (is_found_ap == true && is_arrive_ap_prepoint == false){

                    if (count2<150)
                    {   
                        double delta_pos_x = (ap_pose(0)-appre_pose(0))/150;
                        double delta_pos_y = (ap_pose(1)-appre_pose(1))/150;
                        double delta_pos_z = 0.15 * Clamp_Single(appre_pose(2) - pos_fcu[2], 0.1);
                        cout << "flying to ring post point!" << endl;
                        count2++;
                        for (int i=0;i<6;i++)
                        {
                            nmpc_traj_pt[i].pos(0) = appre_pose(0) + delta_pos_x*count2;
                            nmpc_traj_pt[i].pos(1) = appre_pose(1) + delta_pos_y*count2;
                            nmpc_traj_pt[i].pos(2) = 1.65;
                            nmpc_traj_pt[i].vel(0) = 0.0;
                            nmpc_traj_pt[i].vel(1) = 0.0;
                            nmpc_traj_pt[i].vel(2) = 0.0;
                            cout << "nmpc_traj_pt[i].pos(0) is :" << nmpc_traj_pt[i].pos(0) <<  endl;
                            cout << "nmpc_traj_pt[i].pos(1) is :" << nmpc_traj_pt[i].pos(1) <<  endl;
                            cout << "nmpc_traj_pt[i].pos(2) is :" << nmpc_traj_pt[i].pos(2) <<  endl;
                        }  
                    }
                    else
                    {
                        is_arrive_ap_prepoint = true;
                    }

                    // if (is_send_ap_prepoint == false)
                    // {
                    //     if (search_orient == 0)
                    //     {
                    //         search_orient = 1;
                    //     }
                    //     TypePoint ap_target;
                    //     ap_target.id = Ego_traj_size + search_orient;
                    //     //ap_target.id =1;
                    //     ap_target.mode = 2;
                    //     ap_target.is_map = 0;
                    //     ap_target.x = ap_pose(0);
                    //     ap_target.y = ap_pose(1);
                    //     ap_target.z = 1.28;
                    //     EGO_flag_aimpos(ap_target);
                    //     //ROS_INFO("Flying to Apriltag Prepoint!");
                    //     //ROS_INFO("Publishing point %d", Ego_traj_size+search_orient);
                    //     //ROS_INFO("Flag %d: x: %f, y: %f, z: %f", Ego_traj_size+search_orient, ap_target.x, ap_target.y, ap_target.z);
                    //     ego_flag_pub.publish(flag_traj_msg);
                    //     is_send_ap_prepoint = true;
                    // }
                    // else
                    // {
                    //     PrintInfo(ap_print_count, 20, "Flying to Prepoint!!!");
                    // }
                    // if (flag_state.touch_goal == true)
                    // {
                    //     if (ap_prepoint_lock < 10)
                    //     {
                    //         ap_prepoint_lock++;
                    //     }
                    //     else
                    //     {
                    //         is_arrive_ap_prepoint = true;
                    //         cout << "Arrive Prepoint!" << endl;
                    //     }
                    // }
                    }
                if (is_found_ap == true && is_arrive_ap_prepoint == true) {
                    apland = true;
                    double delta_pos = 0.4 * Clamp_Single(ap_pose(1) - pos_fcu[1], 0.1);
                    for (int i=0;i<6;i++)
                    {
                        nmpc_traj_pt[i].pos(0) = ap_pose(0);
                        nmpc_traj_pt[i].pos(1) = ap_pose(1) + i*delta_pos/3;
                        nmpc_traj_pt[i].pos(2) = 1.65 - 0.005*ctl_land_count;
                        nmpc_traj_pt[i].vel(0) = 0.0;
                        nmpc_traj_pt[i].vel(1) = 0.0;
                        nmpc_traj_pt[i].vel(2) = 0.0;
                    }
                    if (ctl_land_count < 330)
                    {
                        ctl_land_count++;
                        cout << "Landing!!!   " << ctl_land_count << endl;
                        cout << "[Delta_x, Delta_y]: [" << ap_pose(0)-pos_fcu[0] << ", " << ap_pose(1)-pos_fcu[1] << "]" << endl;
                    }
                    // else if (ctl_land_count < 320)
                    // {
                    //     ctl_land_count = ctl_land_count + 1;
                    // }
                    else
                    {
                        if (land_success == false)
                        {
                            cout << "Landing Success!" << endl;
                            cout << "Landing Success!" << endl;
                            cout << "Landing Success!" << endl;
                            land_success = true;
                        }
                    }
                }
            }

            quat_yaw = EulerToQuat(0, 0, YawSmooth(yaw_now, Ego_traj[flag_state.now_id].yaw));
            ROS_INFO("yaw is %f", yaw_now);
            
            geometry_msgs::PoseStamped nmpc_posfdb_msg;
            nmpc_posfdb_msg.pose.position.x = pos_fcu(0);
            nmpc_posfdb_msg.pose.position.y = pos_fcu(1);
            nmpc_posfdb_msg.pose.position.z = pos_fcu(2);
            nmpc_posfdb_pub.publish(nmpc_posfdb_msg);
            std::cout<<"position[0]: "<<pos_fcu(0)<<std::endl;

            geometry_msgs::PoseStamped nmpc_posref_msg;
            nmpc_posref_msg.pose.position.x = nmpc_traj_pt[0].pos(0);
            nmpc_posref_msg.pose.position.y = nmpc_traj_pt[0].pos(1);
            nmpc_posref_msg.pose.position.z = nmpc_traj_pt[0].pos(2);
            nmpc_posref_pub.publish(nmpc_posref_msg);

            // NMPC data update
            nmpc_current_states << pos_fcu(0),pos_fcu(1),pos_fcu(2),vel_fcu(0),vel_fcu(1),vel_fcu(2),quat_fcu.w(),quat_fcu.x(),quat_fcu.y(),quat_fcu.z();
            nmpc_desired_states << 
            nmpc_traj_pt[0].pos(0), nmpc_traj_pt[0].pos(1), nmpc_traj_pt[0].pos(2),
            nmpc_traj_pt[0].vel(0), nmpc_traj_pt[0].vel(1), nmpc_traj_pt[0].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[1].pos(0), nmpc_traj_pt[1].pos(1), nmpc_traj_pt[1].pos(2), 
            nmpc_traj_pt[1].vel(0), nmpc_traj_pt[1].vel(1), nmpc_traj_pt[1].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[2].pos(0), nmpc_traj_pt[2].pos(1), nmpc_traj_pt[2].pos(2), 
            nmpc_traj_pt[2].vel(0), nmpc_traj_pt[2].vel(1), nmpc_traj_pt[2].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[3].pos(0), nmpc_traj_pt[3].pos(1), nmpc_traj_pt[3].pos(2), 
            nmpc_traj_pt[3].vel(0), nmpc_traj_pt[3].vel(1), nmpc_traj_pt[3].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[4].pos(0), nmpc_traj_pt[4].pos(1), nmpc_traj_pt[4].pos(2), 
            nmpc_traj_pt[4].vel(0), nmpc_traj_pt[4].vel(1), nmpc_traj_pt[4].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z(), 
            nmpc_traj_pt[5].pos(0), nmpc_traj_pt[5].pos(1), nmpc_traj_pt[5].pos(2), 
            nmpc_traj_pt[5].vel(0), nmpc_traj_pt[5].vel(1), nmpc_traj_pt[5].vel(2), quat_yaw.w(), quat_yaw.x(), quat_yaw.y(), quat_yaw.z();  //期望的位置
            nmpc_desired_controls << 9.8, 0, 0, 0;                  //期望的控制量
            // NMPC solution
            younger_ctrl.optimal_solution(nmpc_current_states, nmpc_desired_states, nmpc_desired_controls);

            if(younger_ctrl.Acc2Trust())
            {
                mavros_msgs::AttitudeTarget msg;
                msg.header.frame_id = std::string("NMPC");
                msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
                msg.thrust = younger_ctrl.get_control_command()(0);
                msg.body_rate.x = younger_ctrl.get_control_command()(1);
                msg.body_rate.y = younger_ctrl.get_control_command()(2);
                msg.body_rate.z = younger_ctrl.get_control_command()(3);

                std::cout<<"thrust is "<<younger_ctrl.get_control_command()(0)<<std::endl;
                local_attitude_pub.publish(msg);
            }
                
         }
        
        if(cmd == 9)
        {
            mavros_msgs::AttitudeTarget target_arm;
	        target_arm.header.frame_id = std::string("FCU");
            target_arm.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
            target_arm.orientation.x = 0.0;
            target_arm.orientation.y = 0.0;
            target_arm.orientation.z = 0.0;
            target_arm.orientation.w = 1.0;
            target_arm.thrust = nmpc_hover_thrust - 0.03;
            local_attitude_pub.publish(target_arm);
        }

        
        rate.sleep();
    }
    return 0;
}
