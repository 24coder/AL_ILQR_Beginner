#include <cassert>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "ilqr/ilqr_solver.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::ILQROptions;
  using my_al_ilqr::ILQRSolver;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  // 本测试验证基础 iLQR 在非线性 unicycle 任务上的“优化有效性”：
  // - 从零控制初值出发，优化后总代价必须下降；
  // - 终端位置误差应减小，说明优化器不仅改了数值，还改善了任务目标；
  // - cost history 非空，说明迭代流程真实执行。
  auto dynamics = std::make_shared<UnicycleModel>();

  Matrix Q = Matrix::Zero(3, 3);
  Q.diagonal() << 1.0, 1.0, 0.1;
  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.2, 0.05;
  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 20.0, 20.0, 1.0;

  Vector x_ref(3);
  x_ref << 1.5, 0.8, 0.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 30, 0.1);

  Vector x0 = Vector::Zero(3);
  std::vector<Vector> initial_controls(problem.Horizon(), Vector::Zero(2));

  const auto initial_trajectory = problem.Rollout(x0, initial_controls);
  const double initial_cost = problem.TotalCost(initial_trajectory);
  const double initial_position_error =
      (initial_trajectory.State(problem.Horizon()).head(2) - x_ref.head(2)).norm();

  ILQROptions options;
  options.max_iterations = 25;
  options.cost_tolerance = 1e-5;
  options.line_search_max_iterations = 8;
  options.line_search_decrease_factor = 0.5;

  ILQRSolver solver(problem, options);
  const auto optimized_trajectory = solver.Solve(x0, initial_controls);
  const double optimized_cost = problem.TotalCost(optimized_trajectory);
  const double optimized_position_error =
      (optimized_trajectory.State(problem.Horizon()).head(2) - x_ref.head(2)).norm();

  // iLQR 至少应进行一轮有效记录。
  assert(!solver.CostHistory().empty());

  // 核心验证 1：目标值下降，说明数值优化成功推进。
  assert(optimized_cost < initial_cost);

  // 核心验证 2：任务相关指标（终点位置误差）下降，说明优化方向正确。
  assert(optimized_position_error < initial_position_error);

  return 0;
}
