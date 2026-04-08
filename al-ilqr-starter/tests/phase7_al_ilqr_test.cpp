#include <cassert>
#include <memory>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "constraints/control_box_constraint.hpp"
#include "constraints/terminal_goal_constraint.hpp"
#include "cost/quadratic_cost.hpp"
#include "dynamics/linear_point_mass_model.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::ALILQROptions;
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::ConstrainedOptimalControlProblem;
  using my_al_ilqr::ControlBoxConstraint;
  using my_al_ilqr::ILQROptions;
  using my_al_ilqr::LinearPointMassModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::TerminalGoalConstraint;
  using my_al_ilqr::Vector;

  // 本测试验证 AL-iLQR 在受控输入约束 + 终端约束问题上的收敛性：
  // 1) 外层日志非空，说明外层 AL 迭代被执行；
  // 2) 最终约束违反量较初始下降并满足阈值；
  // 3) 收敛标志为真；
  // 4) 终端误差足够小，证明不仅“满足约束”，也达成目标状态。
  auto dynamics = std::make_shared<LinearPointMassModel>();

  Matrix Q = Matrix::Zero(2, 2);
  Q.diagonal() << 0.2, 0.05;
  Matrix R = Matrix::Constant(1, 1, 1e-3);
  Matrix Qf = Matrix::Zero(2, 2);
  Qf.diagonal() << 10.0, 5.0;

  Vector x_ref(2);
  x_ref << 0.5, 0.0;
  Vector u_ref = Vector::Zero(1);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  auto base_problem = std::make_shared<OptimalControlProblem>(dynamics, cost, 20, 0.1);

  ConstrainedOptimalControlProblem constrained_problem(base_problem);
  Vector u_lb(1);
  u_lb << -0.5;
  Vector u_ub(1);
  u_ub << 0.5;
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    constrained_problem.AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(base_problem->StateDim(), u_lb, u_ub));
  }
  constrained_problem.AddTerminalConstraint(std::make_shared<TerminalGoalConstraint>(x_ref));

  Vector x0 = Vector::Zero(2);
  std::vector<Vector> initial_controls(base_problem->Horizon(), Vector::Zero(1));
  for (auto& control : initial_controls) {
    control(0) = 1.0;
  }

  const auto initial_trajectory = base_problem->Rollout(x0, initial_controls);
  initial_trajectory.Print();
  const double initial_violation = constrained_problem.MaxViolation(initial_trajectory);

  ILQROptions inner_options;
  inner_options.max_iterations = 20;
  inner_options.cost_tolerance = 1e-6;
  inner_options.derivative_epsilon = 1e-5;
  inner_options.regularization_init = 1e-6;
  inner_options.regularization_min = 1e-8;
  inner_options.regularization_max = 1e4;
  inner_options.line_search_max_iterations = 10;

  ALILQROptions options;
  options.inner_options = inner_options;
  options.max_outer_iterations = 8;
  options.constraint_tolerance = 2e-3;
  options.initial_penalty = 10.0;
  options.penalty_scaling = 2.0;
  options.penalty_update_ratio = 0.9;
  options.max_penalty = 1e7;

  ALILQRSolver solver(constrained_problem, options);
  const auto result = solver.Solve(x0, initial_controls);
  result.trajectory.Print();

  // 外层确实执行。
  assert(!result.outer_logs.empty());

  const double final_violation = result.outer_logs.back().max_violation;

  // 约束违反应显著降低。
  assert(final_violation < initial_violation);

  // 本案例应达到收敛条件。
  assert(result.converged);

  // 最终违反量应不超过容忍阈值（与配置一致量级）。
  assert(final_violation <= 0.002);

  // 终端状态应贴近目标。
  const Vector terminal_error = result.trajectory.State(base_problem->Horizon()) - x_ref;
  std::cout << "Terminal error: " << terminal_error.transpose() << std::endl;
  assert(terminal_error.cwiseAbs().maxCoeff() <= 0.01);

  return 0;
}
