#include "fsm_ctrl/NMPC_test.hpp"

int states_num = 10;
int controls_num = 4;

NMPC_Ctrller::NMPC_Ctrller(int _predict_step, float _sample_time, double _ctrl_interv, double _hover_thrust,
                           Eigen::Matrix<float, 10, 1> _m_Q_param, Eigen::Matrix<float, 4, 1> _m_R_param)
{
  thrust_to_force = gravity / _hover_thrust;
  ctrl_interv = _ctrl_interv;
  // 设置预测步长和采样时间
  m_predict_step = _predict_step;
  m_sample_time = _sample_time;

  // 初始化预测阶段
  m_predict_stage = -1;

  // 初始化状态量和控制量
  n_states = states_num;      // p v q
  n_controls = controls_num;  // a_BZ omega

  // 初始化初始猜测值
  for (int i = 0; i < m_predict_step; i++)
  {
    m_initial_guess.push_back(0);
    m_initial_guess.push_back(0);
    m_initial_guess.push_back(0);
    m_initial_guess.push_back(0);
  }

  //设置求解器
  set_my_nmpc_solver(_m_Q_param, _m_R_param);
}

void NMPC_Ctrller::set_my_nmpc_solver(Eigen::Matrix<float, 10, 1> _m_Q_param, Eigen::Matrix<float, 4, 1> _m_R_param)
{
  //状态量
  casadi::SX p = casadi::SX::sym("p", 3, 1);  //机体坐标系在世界坐标系下的位置
  casadi::SX v = casadi::SX::sym("v", 3, 1);  //机体坐标系在世界坐标系下的速度
  casadi::SX q = casadi::SX::sym("q", 4, 1);  //机体坐标系在世界坐标系下的姿态四元数

  //控制量
  casadi::SX a_BZ = casadi::SX::sym("a_BZ", 1, 1);    //机体坐标系Z轴的加速度（能够表示推力）
  casadi::SX omega = casadi::SX::sym("omega", 3, 1);  //机体坐标系下的三轴角速度

  //汇总
  casadi::SX x = casadi::SX::vertcat({ p, v, q });
  casadi::SX u = casadi::SX::vertcat({ a_BZ, omega });

  //非线性运动模型
  casadi::SX x_dot = casadi::SX::vertcat({
      v(0),
      v(1),
      v(2),
      2 * (q(0) * q(2) + q(1) * q(3)) * a_BZ,
      2 * (q(2) * q(3) - q(0) * q(1)) * a_BZ,
      (q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3)) * a_BZ - gravity,
      0.5 * (-omega(0) * q(1) - omega(1) * q(2) - omega(2) * q(3)),
      0.5 * (omega(0) * q(0) + omega(2) * q(2) - omega(1) * q(3)),
      0.5 * (omega(1) * q(0) - omega(2) * q(1) + omega(0) * q(3)),
      0.5 * (omega(2) * q(0) + omega(1) * q(1) - omega(0) * q(2)),
  });

  //定义模型函数
  casadi::Function model_function = casadi::Function("f", { x, u }, { x_dot });

  //求解问题符号表示
  casadi::SX U = casadi::SX::sym("U", n_controls, m_predict_step);    //控制输入
  casadi::SX X = casadi::SX::sym("X", n_states, m_predict_step + 1);  //状态输出

  //当前运动状态
  casadi::SX current_states = casadi::SX::sym("current_states", n_states);

  //优化参数(但是这里没有看懂为啥要乘以2)
  //这个参数怎么理解:current states+desired states+desired controls
  casadi::SX opt_param = casadi::SX::sym("opt_param", (2 + m_predict_step) * n_states + n_controls);

  //优化变量
  casadi::SX opt_var = casadi::SX::reshape(U.T(), -1, 1);

  //根据模型函数前向预测无人机运动状态
  X(casadi::Slice(), 0) = opt_param(casadi::Slice(0, 10, 1));  //状态初始值
  for (int i = 0; i < m_predict_step; i++)
  {
    std::vector<casadi::SX> input_X;
    casadi::SX X_current = X(casadi::Slice(), i);
    casadi::SX U_current = U(casadi::Slice(), i);
    input_X.push_back(X_current);
    input_X.push_back(U_current);
    X(casadi::Slice(), i + 1) = model_function(input_X).at(0) * m_sample_time + X_current;
  }

  //预测函数，描述控制序列与输出
  model_predict_function = casadi::Function("predict_function", { casadi::SX::reshape(U, -1, 1), opt_param }, { X });

  //惩罚矩阵
  casadi::SX m_Q = NMPC_Ctrller::CostMatrix_Q(_m_Q_param);
  casadi::SX m_R = NMPC_Ctrller::CostMatrix_R(_m_R_param);

  //计算代价函数
  casadi::SX cost_function = casadi::SX::sym("cost_function");
  cost_function = 0;
  for (int i = 0; i < m_predict_step; i++)
  {
    casadi::SX states_error = X(casadi::Slice(), i) - opt_param(casadi::Slice(10 * (i + 1), 10 * (i + 2), 1));
    casadi::SX control_error = U(casadi::Slice(), i) - opt_param(casadi::Slice(10 * (m_predict_step + 2), 10 * (m_predict_step + 2) + 4, 1));
    cost_function = cost_function + casadi::SX::mtimes({ states_error.T(), m_Q, states_error }) +
                    casadi::SX::mtimes({ control_error.T(), m_R, control_error });
  }

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
  nlp_opts["ipopt.print_level"] = 0;
  nlp_opts["print_time"] = 0;
  nlp_opts["ipopt.acceptable_tol"] = 1e-4;
  nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
  nlp_opts["ipopt.acceptable_dual_inf_tol"] = 1e-4;

  m_solver = nlpsol("solver", solver_name, nlp_problem, nlp_opts);
}

void NMPC_Ctrller::optimal_solution(Eigen::Matrix<float, 10, 1> _current_states,
                                    Eigen::Matrix<float, 60, 1> _desired_states,
                                    Eigen::Matrix<float, 4, 1> _desired_controls)
{
  //控制约束
  std::vector<float> lbx;                   //控制下限
  std::vector<float> ubx;                   //控制上限
  std::vector<float> parameters;            //包括当前状态、目标状态、目标控制
  for (int i = 0; i < m_predict_step; i++)  //加速度限制
  {
    lbx.push_back(-acc_limit);
    ubx.push_back(acc_limit);
  }
  for (int i = 0; i < 3; i++)  //角速度限制
  {
    for (int j = 0; j < m_predict_step; j++)
    {
      lbx.push_back(-omega_limit);
      ubx.push_back(omega_limit);
    }
  }

  for (int i = 0; i < n_states; i++)  //状态约束
  {
    parameters.push_back(_current_states[i]);
  }
  for (int i = 0; i < n_states * (m_predict_step + 1); i++)
  {
    parameters.push_back(_desired_states[i]);
  }
  for (int i = 0; i < n_controls; i++)
  {
    parameters.push_back(_desired_controls[i]);
  }

  //求解参数设置
  m_args["lbx"] = lbx;
  m_args["ubx"] = ubx;
  m_args["p"] = parameters;
  m_args["x0"] = m_initial_guess;

  //求解
  m_result = m_solver(m_args);

  //获取优化变量
  std::vector<float> res_control_all(m_result.at("x"));
  std::vector<float> res_control_a, res_control_omega_roll, res_control_omega_pitch, res_control_omega_yaw;

  res_control_a.assign(res_control_all.begin(), res_control_all.begin() + m_predict_step);
  res_control_omega_roll.assign(res_control_all.begin() + m_predict_step, res_control_all.begin() + 2 * m_predict_step);
  res_control_omega_pitch.assign(res_control_all.begin() + 2 * m_predict_step,
                                 res_control_all.begin() + 3 * m_predict_step);
  res_control_omega_yaw.assign(res_control_all.begin() + 3 * m_predict_step,
                               res_control_all.begin() + 4 * m_predict_step);

  //存储下一时刻最初优化解猜测
  std::vector<float> initial_guess;
  //不知道为啥源代码里少了一次for循环在循环外加的，不是很理解
  //个猜测值代表每次进行控制的初始点位
  for (int i = 0; i < m_predict_step; i++)
  {
    initial_guess.push_back(res_control_a.at(i));
  }
  for (int i = 0; i < m_predict_step; i++)
  {
    initial_guess.push_back(res_control_omega_roll.at(i));
  }
  for (int i = 0; i < m_predict_step; i++)
  {
    initial_guess.push_back(res_control_omega_pitch.at(i));
  }
  for (int i = 0; i < m_predict_step; i++)
  {
    initial_guess.push_back(res_control_omega_yaw.at(i));
  }

  m_initial_guess = initial_guess;

  //控制序列的第一组作为当前控制量
  m_control_command << res_control_a.front(), res_control_omega_roll.front(), res_control_omega_pitch.front(),
      res_control_omega_yaw.front();

  // std::cout << "m_control_command: " << m_control_command << std::endl;
}

/**
 * @description:推力估计
 * @param {Vector3d&} _acc
 * @return {*}推力估计是否成功
 */
bool NMPC_Ctrller::EstimateThrust(Eigen::Vector3d& _acc)
{
  ros::Time now = ros::Time::now();
  while (thrust_stamped.size() >= 1)
  {
    std::pair<ros::Time, double> time_thrust = thrust_stamped.front();
    double time_pass = (now - time_thrust.first).toSec();

    /*    选择2循环前的数据进行递推更新    */
    if (time_pass > 3 * ctrl_interv)
    {
      thrust_stamped.pop();
      std::cout << "time_pass > 3*ctrl_interv!!" << std::endl;
      continue;
    }
    else if (time_pass < ctrl_interv)
    {
      std::cout << "time_pass < ctrl_interv!!" << std::endl;
      return false;
    }

    /*   渐消记忆的递推最小二乘估计 估计模型为force = thrust_to_force * thrust   */
    double thrust = time_thrust.second;  //推力和加速度是反向的
    thrust_stamped.pop();
    double gamma = 1 / (1.0 + thrust * p_estimate * thrust);
    double k_estimate = gamma * p_estimate * thrust;

    thrust_to_force = thrust_to_force + k_estimate * (_acc.norm() - thrust * thrust_to_force);
    p_estimate = (1 - k_estimate * thrust) * p_estimate / 1.0;
    // std::cout << "thrust: " << thrust << std::endl;
    // std::cout << "p_estimate: " << p_estimate << std::endl;
    // std::cout << "k_estimate: " << k_estimate << std::endl;
    // std::cout << "gamma: " << gamma << std::endl;
    // std::cout << "thrust_to_force: " << thrust_to_force << std::endl;
    return true;
  }
  return false;
}

bool NMPC_Ctrller::Acc2Trust()
{
  double control_desired_thrust = m_control_command(0) / thrust_to_force;
  thrust_stamped.push(std::pair<ros::Time, double>(ros::Time::now(), control_desired_thrust));
  while (thrust_stamped.size() > 100)
  {
    thrust_stamped.pop();
  }
  Eigen::Vector3d control_desired_acceleration;
  control_desired_acceleration[0] = control_desired_acceleration[1] = 0;
  control_desired_acceleration[2] = m_control_command(0);

  if (EstimateThrust(control_desired_acceleration))
  {
    m_control_command(0) = control_desired_thrust;
    return true;
  }
  else
    return false;
}

Eigen::Matrix<float, 4, 1> NMPC_Ctrller::get_control_command()
{
  return m_control_command;
}

casadi::SX NMPC_Ctrller::CostMatrix_Q(Eigen::Matrix<float, 10, 1> _m_Q_param)
{
  return casadi::SX::diag(
      { casadi::SX::vertcat({ _m_Q_param(0), _m_Q_param(1), _m_Q_param(2), _m_Q_param(3), _m_Q_param(4), _m_Q_param(5),
                              _m_Q_param(6), _m_Q_param(7), _m_Q_param(8), _m_Q_param(9) }) });
}

casadi::SX NMPC_Ctrller::CostMatrix_R(Eigen::Matrix<float, 4, 1> _m_R_param)
{
  return casadi::SX::diag({ casadi::SX::vertcat({ _m_R_param(0), _m_R_param(1), _m_R_param(2), _m_R_param(3) }) });
}
