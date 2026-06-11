/**
 * @file    att_nmpc
 * @brief   NMPC high-level control from position + yaw command to thrust + attitude command
 * @author  FLAG Lab, BIT
 * @version 1.0
 * @date    2024-07-07
 */

#include <fsm_ctrl/att_nmpc.hpp>

using namespace casadi;



/**
 * @brief  NMPC class constructor
 * @param  _mat_Q, _mat_R: state & input weight matrix
 * @param  _acc_limit, _att_limit: acceleration & attitude limit
 * @param  _ctrl_interv: control interval
 * @param  _hover_thrust: hover thrust
 * @return NULL
 */
NMPC::NMPC(Eigen::Matrix<double, STATE_NUM, 1> _mat_Q, Eigen::Matrix<double, INPUT_NUM, 1> _mat_R, 
           double _acc_limit, double _att_limit, int _ctrl_rate, double _hover_thrust)
{
    mat_Q = _mat_Q;
    mat_R = _mat_R;
    acc_limit = _acc_limit;
    att_limit = _att_limit;
    /* initialize control input as hover thrust + attitude */
    for(int i = 0; i < STEP; i++) {init_solution.push_back(GRAVITY);}
    for(int i = 0; i < 3*STEP; i++) {init_solution.push_back(0.0);}
    
    Set_Solver();
    thrust_estor.Set_Estor(1.0/_ctrl_rate, _hover_thrust);
}


/**
 * @brief  CasADi solver setup
 * @param  NULL
 * @return NULL
 */
void NMPC::Set_Solver()
{
    /* state */
    SX pos = SX::sym("pos", 3);         // position in world frame
    SX vel = SX::sym("vel", 3);         // velocity in world frame
    SX yaw = SX::sym("yaw", 1);         // yaw angle from world frame to body frame
    SX state = SX::vertcat({pos, vel, yaw});

    /* input */
    SX acc_zb = SX::sym("acc_zb", 1);             // acceleration on z-axis in body frame
    SX tilt = SX::sym("tilt", 2);                 // tilt angle from world frame to body frame
    SX yaw_rate = SX::sym("yaw_rate", 1);         // yaw rate from world frame to body frame
    SX input = SX::vertcat({acc_zb, tilt, yaw_rate});

    /* dynamics model */
    SX state_dot = SX::vertcat({
        vel(0),
        vel(1),
        vel(2),
        (cos(yaw)*cos(tilt(0))*sin(tilt(1)) + sin(yaw)*sin(tilt(0)))*acc_zb,
        (sin(yaw)*cos(tilt(0))*sin(tilt(1)) - cos(yaw)*sin(tilt(0)))*acc_zb,
        (cos(tilt(0))*cos(tilt(1)))*acc_zb - GRAVITY,
        yaw_rate
    });
    Function model_func = Function("model_func", {state, input}, {state_dot});
  
    /* full variables in prediction steps */
    SX X = SX::sym("X", STATE_NUM, STEP + 1);
    SX U = SX::sym("U", INPUT_NUM, STEP);

    /* optimal parameters = current state, (STEP + 1) desired states, desired input */
    SX opt_param = SX::sym("opt_param", (STEP + 2)*STATE_NUM + INPUT_NUM);

    /* model prediction */
    X(Slice(), 0) = opt_param(Slice(0, STATE_NUM, 1));         // initial state
    for (int i = 0; i < STEP; i++)
    {
        std::vector<SX> state_input_vec;
        SX state_current = X(Slice(), i);
        SX input_current = U(Slice(), i);
        state_input_vec.push_back(state_current);
        state_input_vec.push_back(input_current);
        X(Slice(), i + 1) = model_func(state_input_vec).at(0)*INTERV + state_current;
    }

    /* cost matrix */
    SX Q = SX::diag({SX::vertcat({mat_Q(0), mat_Q(1), mat_Q(2), mat_Q(3), mat_Q(4), mat_Q(5), mat_Q(6)})});
    SX R = SX::diag({SX::vertcat({mat_R(0), mat_R(1), mat_R(2), mat_R(3)})});

    /* calculate cost */
    SX cost_func = SX::sym("cost_func", 1);
    cost_func = 0;
    for (int i = 0; i < STEP; i++)
    {
        SX state_error = opt_param(Slice(STATE_NUM*(i + 1), STATE_NUM*(i + 2), 1)) - X(Slice(), i); 
        SX input_error = opt_param(Slice(STATE_NUM*(STEP + 2), STATE_NUM*(STEP + 2) + 4, 1))- U(Slice(), i);
        cost_func = cost_func + SX::mtimes({state_error.T(), Q, state_error}) + SX::mtimes({input_error.T(), R, input_error});
    }

    /* optimal variables */
    SX opt_var = SX::reshape(U.T(), -1, 1);

    /* build slover */
    SXDict nlp_prob = {{"f", cost_func}, {"x", opt_var}, {"p", opt_param}};

    Dict nlp_opt;
    nlp_opt["expand"] = true;
    nlp_opt["ipopt.max_iter"] = 5000;
    nlp_opt["ipopt.print_level"] = 0;
    nlp_opt["print_time"] = 0;
    nlp_opt["ipopt.acceptable_tol"] = 1e-4;
    nlp_opt["ipopt.acceptable_obj_change_tol"] = 1e-4;
    nlp_opt["ipopt.acceptable_dual_inf_tol"] = 1e-4;

    solver = nlpsol("solver", "ipopt", nlp_prob, nlp_opt);
}


/**
 * @brief  solve NMPC
 * @param  _pod_fdb, _vel_fdb: position feedback
 * @param  _yaw_fdb: yaw feedback
 * @param  _state_ref: state reference of all steps
 * @return mavros_msgs::AttitudeTarget command 
 */
mavros_msgs::AttitudeTarget NMPC::OptSolve(Eigen::Vector3d _pos_fdb, Eigen::Vector3d _vel_fdb, double _yaw_fdb, std::vector<CtrlPt> _state_ref)
{
    mavros_msgs::AttitudeTarget cmd;
    if(_state_ref.empty())
    {
        cmd = ArmCmd();
        return cmd;
    }
    
    std::vector<double> lbx;           // optimal variable lower bound
    std::vector<double> ubx;           // optimal variable upper bound
    std::vector<double> param;         // current state + desired states of all steps + desired input
    
    /* set optimal bounds */
    for (int i = 0; i < STEP; i++)
    {
        lbx.push_back(-acc_limit);
        ubx.push_back(acc_limit);
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < STEP; j++)
        {
            lbx.push_back(-att_limit);
            ubx.push_back(att_limit);
        }
    }

    /* set optimal parameters */
    for(int i = 0; i < 3; i++) {param.push_back(_pos_fdb(i));}
    for(int i = 0; i < 3; i++) {param.push_back(_vel_fdb(i));}
    param.push_back(_yaw_fdb);
    
    for (int j = 0; j < (STEP + 1); j++)
    {
        for(int i = 0; i < 3; i++) {param.push_back(_state_ref[j].pos(i));}
        for(int i = 0; i < 3; i++) {param.push_back(_state_ref[j].vel(i));}
        param.push_back(_state_ref[j].euler(2));
    }

    param.push_back(GRAVITY);
    for(int i = 0; i < 3; i++) {param.push_back(0.0);}

    /* set solver */
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["p"] = param;
    arg["x0"] = init_solution;

    /* solve */
    solution = solver(arg);

    /* get control input from solution */
    std::vector<double> opt_solution(solution.at("x"));
    std::vector<double> input_acc_zb, input_roll, input_pitch, input_yaw_rate;
    input_acc_zb.assign(opt_solution.begin(), opt_solution.begin() + STEP);
    input_roll.assign(opt_solution.begin() + STEP, opt_solution.begin() + 2*STEP);
    input_pitch.assign(opt_solution.begin() + 2*STEP, opt_solution.begin() + 3*STEP);
    input_yaw_rate.assign(opt_solution.begin() + 3*STEP, opt_solution.begin() + 4*STEP);

    /* set initial optimal varible */
    std::vector<double> init_guess;
    for (int i = 0; i < STEP; i++) {init_guess.push_back(input_acc_zb.at(i));}
    for (int i = 0; i < STEP; i++) {init_guess.push_back(input_roll.at(i));}
    for (int i = 0; i < STEP; i++) {init_guess.push_back(input_pitch.at(i));}
    for (int i = 0; i < STEP; i++) {init_guess.push_back(input_yaw_rate.at(i));}
    init_solution = init_guess;

    /* the first input used for control command */
    solution_input << input_acc_zb.front(), input_roll.front(), input_pitch.front(), input_yaw_rate.front();
    cmd = AttCmd(_state_ref.front().euler(2));
    return cmd;
}


/**
 * @brief  generate attitude control command
 * @param  _yaw yaw reference
 * @return mavros_msgs::AttitudeTarget command
 */
mavros_msgs::AttitudeTarget NMPC::AttCmd(double _yaw)
{
	double thrust = thrust_estor.LSE(solution_input[0]);
    Eigen::Quaterniond quat = EulerToQuat(solution_input[1], solution_input[2], _yaw);
    mavros_msgs::AttitudeTarget msg;
	msg.header.frame_id = std::string("FCU");
	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
	msg.thrust = thrust;
    msg.orientation.w = quat.w();
    msg.orientation.x = quat.x();
	msg.orientation.y = quat.y();
	msg.orientation.z = quat.z();
    return msg;
}