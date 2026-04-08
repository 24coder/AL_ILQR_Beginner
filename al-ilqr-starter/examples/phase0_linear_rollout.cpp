#include <iostream>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/linear_point_mass_model.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  // Phase 0：最小可运行示例（只做前向 rollout，不做优化）。
  // 目的：帮助理解“给定动力学 + 给定控制序列 -> 状态轨迹 + 累计代价”这一基础闭环。
  using my_al_ilqr::LinearPointMassModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<LinearPointMassModel>();

  // 代价权重：Q/R 控制过程中的状态偏差与控制能量，Qf 控制终端状态误差。
  Matrix Q = Matrix::Zero(2, 2);
  Q(0, 0) = 1.0;
  Q(1, 1) = 0.1;

  Matrix R = Matrix::Constant(1, 1, 0.05);

  Matrix Qf = Matrix::Zero(2, 2);
  Qf(0, 0) = 10.0;
  Qf(1, 1) = 1.0;

  // 参考状态：希望位置到 1.5，速度回到 0。
  Vector x_ref(2);
  x_ref << 1.5, 0.0;
  Vector u_ref = Vector::Zero(1);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  // 时域参数：horizon=20，dt=0.1s，总时长约 2 秒。
  OptimalControlProblem problem(dynamics, cost, 20, 0.1);

  // 阶段流程：
  // 1) 构造初始状态 x0；
  // 2) 手工给出一组开环控制；
  // 3) rollout 得到轨迹；
  // 4) 计算总成本并打印关键采样点。
  Vector x0 = Vector::Zero(2);
  std::vector<Vector> controls(problem.Horizon(), Vector::Zero(1));
  for (int k = 0; k < problem.Horizon(); ++k) {
    // 前半段加速、后半段减速，用于直观看到位置推进与速度回落。
    controls[k](0) = (k < 10) ? 0.4 : -0.2;
  }

  const auto trajectory = problem.Rollout(x0, controls);
  const double total_cost = problem.TotalCost(trajectory);

  std::cout << "Phase 0 linear rollout example\n";
  std::cout << "horizon = " << problem.Horizon() << ", dt = " << problem.TimeStep() << "\n";
  std::cout << "first states:\n";
  for (int k = 0; k < 5; ++k) {
    std::cout << "  x[" << k << "] = " << trajectory.State(k).transpose() << "\n";
  }
  std::cout << "terminal state = " << trajectory.State(problem.Horizon()).transpose() << "\n";
  std::cout << "total cost = " << total_cost << "\n";

  return 0;
}
