#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "problems/optimal_control_problem.hpp"

namespace {

bool NearlyEqual(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  // 本测试验证 unicycle 模型在“直行”控制下的动力学正确性：
  // - 输入 v=1, omega=0，初始航向 0；
  // - 预期仅 x 方向前进，y 与航向保持不变；
  // - 同时确认代价函数能对该轨迹给出有效（正）代价。
  auto dynamics = std::make_shared<UnicycleModel>();
  Matrix Q = Matrix::Identity(3, 3);
  Matrix R = Matrix::Identity(2, 2);
  Matrix Qf = 2.0 * Matrix::Identity(3, 3);
  Vector x_ref = Vector::Zero(3);
  Vector u_ref = Vector::Zero(2);
  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 5, 0.2);

  Vector x0 = Vector::Zero(3);
  std::vector<Vector> controls(problem.Horizon(), Vector::Zero(2));
  for (Vector& u : controls) {
    u(0) = 1.0;
    u(1) = 0.0;
  }

  const auto trajectory = problem.Rollout(x0, controls);

  // 验证最终位姿，确保运动学离散更新逻辑正确。
  assert(NearlyEqual(trajectory.State(problem.Horizon())(0), 1.0));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(1), 0.0));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(2), 0.0));

  // 验证该轨迹在二次型代价下可被正常评估。
  assert(problem.TotalCost(trajectory) > 0.0);

  return 0;
}
