#include <iostream>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  // Phase 1（Unicycle）：在非线性单轨模型上做 rollout，展示“曲线轨迹由控制决定”。
  // 相比 Phase 0，本示例重点是姿态角 yaw 与角速度控制对平面轨迹的影响。
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<UnicycleModel>();

  // 关键参数：Q/Qf 更重视 x、y 收敛，R 约束速度与转向控制幅度。
  Matrix Q = Matrix::Zero(3, 3);
  Q.diagonal() << 1.0, 1.0, 0.2;
  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.1, 0.05;
  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 8.0, 8.0, 1.0;

  Vector x_ref(3);
  x_ref << 2.0, 1.5, 0.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  // 时域：30 步，每步 0.1s。
  OptimalControlProblem problem(dynamics, cost, 30, 0.1);

  Vector x0 = Vector::Zero(3);
  std::vector<Vector> controls(problem.Horizon(), Vector::Zero(2));
  for (int k = 0; k < problem.Horizon(); ++k) {
    // 控制设计：保持前进速度，前段左转、后段轻微右转，形成 S 型趋势。
    controls[k](0) = 0.8;
    controls[k](1) = (k < 12) ? 0.35 : -0.1;
  }

  const auto trajectory = problem.Rollout(x0, controls);

  std::cout << "Phase 1 unicycle rollout example\n";
  std::cout << "state_dim = " << problem.StateDim()
            << ", control_dim = " << problem.ControlDim()
            << ", horizon = " << problem.Horizon() << "\n";
  std::cout << "sampled states:\n";
  for (int k : {0, 5, 10, 15, 20, 30}) {
    std::cout << "  x[" << k << "] = " << trajectory.State(k).transpose() << "\n";
  }
  std::cout << "total cost = " << problem.TotalCost(trajectory) << "\n";

  return 0;
}
