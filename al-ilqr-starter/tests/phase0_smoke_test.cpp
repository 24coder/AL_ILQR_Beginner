#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/linear_point_mass_model.hpp"
#include "problems/optimal_control_problem.hpp"

namespace {

bool NearlyEqual(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  using my_al_ilqr::LinearPointMassModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::Vector;

  // 该 smoke test 验证最基础的“动力学 rollout + 代价计算”流水线是否可用：
  // 1) 线性点质量模型可被问题对象正确调用；
  // 2) 轨迹尺寸（状态/控制维度与时域长度）正确；
  // 3) 在常值控制下终端状态与解析预期一致；
  // 4) 总代价为正，说明代价项已被正常累计。
  auto dynamics = std::make_shared<LinearPointMassModel>();

  Matrix Q = Matrix::Identity(2, 2);
  Matrix R = Matrix::Identity(1, 1);
  Matrix Qf = 5.0 * Matrix::Identity(2, 2);
  Vector x_ref = Vector::Zero(2);
  Vector u_ref = Vector::Zero(1);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 5, 0.2);

  Vector x0 = Vector::Zero(2);
  std::vector<Vector> controls(problem.Horizon(), Vector::Ones(1));

  const auto trajectory = problem.Rollout(x0, controls);
  const double total_cost = problem.TotalCost(trajectory);

  // 验证轨迹基本形状，确保数据容器构建正确。
  assert(trajectory.Horizon() == 5);
  assert(trajectory.StateDim() == 2);
  assert(trajectory.ControlDim() == 1);

  // 验证离散积分后的终端状态：常值加速度输入下 x≈0.5, v≈1.0。
  assert(NearlyEqual(trajectory.State(problem.Horizon())(0), 0.5));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(1), 1.0));

  // 验证代价计算已连通（正定权重下非零轨迹应给出正代价）。
  assert(total_cost > 0.0);

  return 0;
}
