#include "fsm_ctrl/NMPC_Controller.hpp"

NMPC_Ctrller_simple::NMPC_Ctrller_simple(double _ctrl_T, 
                                         std::array<double, 2> _acc_z_limit, 
                                         std::array<double, 2> _w_limit,
                                         int _NLP_predict_step, 
                                         double _NLP_onestep_time, 
                                         int _NLP_state_num, 
                                         int _NLP_input_num,
                                         Eigen::Matrix<float, 3, 1> _NLP_costQ_pos,
                                         Eigen::Matrix<float, 3, 1> _NLP_costQ_vel,
                                         Eigen::Matrix<float, 3, 1> _NLP_costQ_quat,
                                         Eigen::Matrix<float, 3, 1> _NLP_costR_w,
                                         double _NLP_costR_acc_z,
                                         double _hover_thrust)
{
    //固定参数赋值，包括控制器周期，输入约束，状态约束，NLP问题求解步数，单步时长，状态变量数目，输入变量数目，代价函数系数
    ctrl_T = _ctrl_T; //
    acc_z_limit = _acc_z_limit; //
    w_limit = _w_limit; //
    NLP_predict_step = _NLP_predict_step; //
    NLP_onestep_time = _NLP_onestep_time; //
    NLP_state_num = _NLP_state_num; //
    NLP_input_num = _NLP_input_num; //
    NLP_costQ_pos = _NLP_costQ_pos;
    NLP_costQ_vel = _NLP_costQ_vel;
    NLP_costQ_quat = _NLP_costQ_quat;
    NLP_costR_w = _NLP_costR_w;
    NLP_costR_acc_z = _NLP_costR_acc_z;

    thr_est.Set_Estor(50, _hover_thrust);

    // 初始化初始猜测值
    for (int i = 0; i < NLP_input_num * NLP_predict_step; i++)
    {NLP_initial_u0.push_back(0);}

    /* states */
    casadi::SX p = casadi::SX::sym("p", 3);             // position in world frame 3*1
    casadi::SX v = casadi::SX::sym("v", 3);             // velocity in world frame 3*1
    casadi::SX q = casadi::SX::sym("q", 4);             // attitude quaternion from world frame to body frame 3*1
    /* inputs */
    casadi::SX w = casadi::SX::sym("w", 3);             // angle velocity in body frame 3*1
    casadi::SX acc_z = casadi::SX::sym("acc_z", 1);             // motor thrust in body frame 4*1

    /* total states and total inputs */
    casadi::SX x = casadi::SX::vertcat({p, v, q}); // x:10*1
    casadi::SX u = casadi::SX::vertcat({w, acc_z}); // u:4*1

    /* dynamics model */
    casadi::SX x_dot = casadi::SX::vertcat({
        v(0),
        v(1),
        v(2),
        2*(q(0)*q(2) + q(1)*q(3))* acc_z,
        2*(q(2)*q(3) - q(0)*q(1))* acc_z,
        (q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3))* acc_z- world_gravity_acc,
        0.5*(-q(1)*w(0) - q(2)*w(1) - q(3)*w(2)),
        0.5*(q(0)*w(0) + q(2)*w(2) - q(3)*w(1)),
        0.5*(q(0)*w(1) - q(1)*w(2) + q(3)*w(0)),
        0.5*(q(0)*w(2) + q(1)*w(1) - q(2)*w(0)),
    }); // x_dot:13*1

    //定义模型函数
    casadi::Function model_function = casadi::Function("f", {x, u}, {x_dot});

    //求解问题符号表示
    casadi::SX U = casadi::SX::sym("U", NLP_input_num, NLP_predict_step);  //控制输入，本案例中为 4*(predict_step)
    casadi::SX X = casadi::SX::sym("X", NLP_state_num, NLP_predict_step + 1);  //状态输出，本案例中为 13*(predict_step+1)

    //优化问题所需参数 当前所有状态+期望轨迹点对应位置、速度、姿态、加速度+期望控制量 10+(1+predict_step)*10+predict_step*4
    casadi::SX opt_param = casadi::SX::sym("opt_param", (NLP_predict_step + 2) * NLP_state_num + NLP_predict_step * NLP_input_num);
    // std::cout << "SX 变量 opt_param:" << opt_param << std::endl;

    //优化变量 (predict_step*input_num)*1，从上到下分别为第一个控制量的predict_step步，第二个控制量的predict_step步，...，第input_num个控制量的predict_step步
    casadi::SX opt_var = casadi::SX::reshape(U.T(), -1, 1);
    // std::cout << "SX 变量 opt_var:" << opt_var << std::endl;

    //根据模型函数前向预测无人机运动状态
    X(casadi::Slice(), 0) = opt_param(casadi::Slice(0, NLP_state_num, 1));  //状态初始值

    for (int i = 0; i < NLP_predict_step; i++)
    {
        std::vector<casadi::SX> input_X; //最终为10*1
        casadi::SX X_current = X(casadi::Slice(), i);
        casadi::SX U_current = casadi::SX::vertcat({opt_var(i),opt_var(i+NLP_predict_step),opt_var(i+2*NLP_predict_step),opt_var(i+3*NLP_predict_step)});
        // std::cout << "SX 变量 U_current:" << U_current << std::endl;
        input_X.push_back(X_current);
        input_X.push_back(U_current);
        X(casadi::Slice(), i + 1) = model_function(input_X).at(0) * NLP_onestep_time + X_current;
    }

    //惩罚矩阵
    casadi::SX costQpos = casadi::SX::diag({casadi::SX::vertcat({ NLP_costQ_pos(0), NLP_costQ_pos(1), NLP_costQ_pos(2)})});
    casadi::SX costQvel = casadi::SX::diag({casadi::SX::vertcat({ NLP_costQ_vel(0), NLP_costQ_vel(1), NLP_costQ_vel(2)})});
    casadi::SX costQquat = casadi::SX::diag({casadi::SX::vertcat({ NLP_costQ_quat(0), NLP_costQ_quat(1), NLP_costQ_quat(2)})});
    casadi::SX costRw = casadi::SX::diag({casadi::SX::vertcat({ NLP_costR_w(0), NLP_costR_w(1), NLP_costR_w(2)})});

    //计算代价函数
    casadi::SX cost_function = casadi::SX::sym("cost_function");
    cost_function = 0;
    for (int i = 0; i < NLP_predict_step; i++)
    {
        casadi::SX error_pos = opt_param(casadi::Slice(NLP_state_num*(i+1), NLP_state_num*(i+1)+3, 1)) - X(casadi::Slice(0,3,1), i);
        casadi::SX error_vel = opt_param(casadi::Slice(NLP_state_num*(i+1)+3, NLP_state_num*(i+1)+6, 1)) - X(casadi::Slice(3,6,1), i);
        casadi::SX error_quat = casadi::SX::vertcat({X(6, i)*opt_param(NLP_state_num*(i+1)+7) - X(7, i)*opt_param(NLP_state_num*(i+1)+6) + X(8, i)*opt_param(NLP_state_num*(i+1)+9) - X(9, i)*opt_param(NLP_state_num*(i+1)+8),
                                                     X(6, i)*opt_param(NLP_state_num*(i+1)+8) - X(7, i)*opt_param(NLP_state_num*(i+1)+9) - X(8, i)*opt_param(NLP_state_num*(i+1)+6) + X(9, i)*opt_param(NLP_state_num*(i+1)+7),
                                                     X(6, i)*opt_param(NLP_state_num*(i+1)+9) + X(7, i)*opt_param(NLP_state_num*(i+1)+8) - X(8, i)*opt_param(NLP_state_num*(i+1)+7) - X(9, i)*opt_param(NLP_state_num*(i+1)+6)});
        casadi::SX error_w = opt_param(casadi::Slice((NLP_state_num*(NLP_predict_step+2)+NLP_input_num*i), (NLP_state_num*(NLP_predict_step+2)+NLP_input_num*i+3), 1)) - U(casadi::Slice(0,3,1), i);
        casadi::SX error_acc_z = opt_param(NLP_state_num*(NLP_predict_step+2)+NLP_input_num*i+3) - U(3, i);
        cost_function = cost_function + casadi::SX::mtimes({ error_pos.T(), costQpos, error_pos }) +
                                        casadi::SX::mtimes({ error_vel.T(), costQvel, error_vel }) + 
                                        casadi::SX::mtimes({ error_quat.T(), costQquat, error_quat }) + 
                                        casadi::SX::mtimes({ error_w.T(), costRw, error_w }) +
                                        NLP_costR_acc_z * error_acc_z * error_acc_z;
    }

    casadi::SX error_pos = opt_param(casadi::Slice(NLP_state_num*(NLP_predict_step+1), NLP_state_num*(NLP_predict_step+1)+3, 1)) - X(casadi::Slice(0,3,1), NLP_predict_step);
    casadi::SX error_vel = opt_param(casadi::Slice(NLP_state_num*(NLP_predict_step+1)+3, NLP_state_num*(NLP_predict_step+1)+6, 1)) - X(casadi::Slice(3,6,1), NLP_predict_step);
    casadi::SX error_quat = casadi::SX::vertcat({X(6, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+7) - X(7, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+6) + X(8, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+9) - X(9, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+8),
                                                 X(6, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+8) - X(7, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+9) - X(8, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+6) + X(9, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+7),
                                                 X(6, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+9) + X(7, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+8) - X(8, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+7) - X(9, NLP_predict_step)*opt_param(NLP_state_num*(NLP_predict_step+1)+6)});
    cost_function = cost_function + casadi::SX::mtimes({ error_pos.T(), costQpos, error_pos }) +
                                    casadi::SX::mtimes({ error_vel.T(), costQvel, error_vel }) + 
                                    casadi::SX::mtimes({ error_quat.T(), costQquat, error_quat });    

    //构建求解器（不考虑优化问题）
    //这里的变量说明可以找Casadi C++ API手册
    casadi::SXDict nlp_problem = {
        { "f", cost_function },
        { "x", opt_var },    //系统输出
        { "p", opt_param },  //优化参数,即当前状态、目标状态、目标控制
    };

    std::string solver_name = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 5000;
    nlp_opts["ipopt.print_level"] = 5;
    nlp_opts["print_time"] = 1;
    nlp_opts["ipopt.acceptable_tol"] = 1e-4;
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
    nlp_opts["ipopt.acceptable_dual_inf_tol"] = 1e-4;

    NLP_solver = nlpsol("solver", solver_name, nlp_problem, nlp_opts);
}

NMPC_Ctrller_simple& NMPC_Ctrller_simple::operator=(const NMPC_Ctrller_simple& other)
{
    if (this != &other)
    {
        ctrl_T = other.ctrl_T; //
        acc_z_limit = other.acc_z_limit; //
        w_limit = other.w_limit; //
        NLP_predict_step = other.NLP_predict_step; //
        NLP_onestep_time = other.NLP_onestep_time; //
        NLP_state_num = other.NLP_state_num; //
        NLP_input_num = other.NLP_input_num; //
        NLP_costQ_pos = other.NLP_costQ_pos; //
        NLP_costQ_vel = other.NLP_costQ_vel; //
        NLP_costQ_quat = other.NLP_costQ_quat; //
        NLP_costR_w = other.NLP_costR_w; //
        NLP_costR_acc_z = other.NLP_costR_acc_z; //
    }
    return *this;
}

void NMPC_Ctrller_simple::optimal_solution(std::vector<double> _current_states,
                                           std::vector<double> _desired_params)
{
    size_t current_states_size = static_cast<size_t>(NLP_state_num);
    size_t desired_params_size = static_cast<size_t>((NLP_predict_step + 1) * NLP_state_num + NLP_predict_step * NLP_input_num);
    //向量大小检查
    if (_current_states.size() != current_states_size) 
    {
        std::cerr << "Warning! Current States Vector size changes!" << std::endl;
    }
    if (_desired_params.size() != desired_params_size) 
    {
        std::cerr << "Warning! Desired Params Vector size changes!" << std::endl;
    }

    //控制约束
    std::vector<double> lbx;                   //控制下限
    std::vector<double> ubx;                   //控制上限
    std::vector<double> parameters;            //包括当前状态、目标状态、目标控制

    for(int i = 0; i < NLP_input_num - 1; i++)
    {
        for(int j = 0; j < NLP_predict_step; j++)
        {
            lbx.push_back(w_limit.at(0));
            ubx.push_back(w_limit.at(1));
        }
    }
    for (int j = 0; j < NLP_predict_step; j++)  //力限制
    {
        lbx.push_back(acc_z_limit.at(0));
        ubx.push_back(acc_z_limit.at(1));
    }

    // size_t current_states_size = _current_states.size();
    // size_t desired_params_size = _desired_params.size();
    // std::cout << "current_states_size" << lbx.size() << std::endl;

    for (size_t i = 0; i < current_states_size; i++)  //传入参数
    {
        parameters.push_back(_current_states[i]);
    }
    for (size_t i = 0; i < desired_params_size; i++)
    {
        parameters.push_back(_desired_params[i]);
    }
    // std::cout << parameters << std::endl;

    //求解参数设置
    NLP_args["lbx"] = lbx;
    NLP_args["ubx"] = ubx;
    NLP_args["p"] = parameters;
    NLP_args["x0"] = NLP_initial_u0;

    //求解
    NLP_result = NLP_solver(NLP_args);

    //获取优化变量
    std::vector<double> cmd_control_all(NLP_result.at("x"));
    optimal_command = cmd_control_all;
    // std::cout << "cmd_control_all: " << cmd_control_all << std::endl;
    std::vector<double> cmd_control_wx, cmd_control_wy, cmd_control_wz, cmd_control_acc_z;
    cmd_control_wx.assign(cmd_control_all.begin(), cmd_control_all.begin() + NLP_predict_step);
    cmd_control_wy.assign(cmd_control_all.begin() + NLP_predict_step,
                          cmd_control_all.begin() + 2 * NLP_predict_step);
    cmd_control_wz.assign(cmd_control_all.begin() + 2 * NLP_predict_step,
                          cmd_control_all.begin() + 3 * NLP_predict_step);
    cmd_control_acc_z.assign(cmd_control_all.begin() + 3 * NLP_predict_step,
                              cmd_control_all.begin() + 4 * NLP_predict_step);

    //存储下一时刻最初优化解猜测
    std::vector<double> initial_guess;

    for (int i = 0; i < NLP_predict_step; i++)
    {
        initial_guess.push_back(cmd_control_wx.at(i));
    }
    for (int i = 0; i < NLP_predict_step; i++)
    {
        initial_guess.push_back(cmd_control_wy.at(i));
    }
    for (int i = 0; i < NLP_predict_step; i++)
    {
        initial_guess.push_back(cmd_control_wz.at(i));
    }
    for (int i = 0; i < NLP_predict_step; i++)
    {
        initial_guess.push_back(cmd_control_acc_z.at(i));
    }

    NLP_initial_u0 = initial_guess;

    //控制序列的第一组作为当前控制量
    w_command << cmd_control_wx.front(), cmd_control_wy.front(), cmd_control_wz.front();
    acc_z_command = thr_est.LSE(cmd_control_acc_z.front());
    // std::cout << "acc_z_command: " << acc_z_command << std::endl;
}
