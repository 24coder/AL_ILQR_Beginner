#include <cassert>
#include <cmath>

#include "core/types.hpp"
#include "lqr/finite_horizon_lqr_solver.hpp"

namespace {

bool NearlyEqual(double a, double b, double tol = 1e-6) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  using my_al_ilqr::FiniteHorizonLQRProblem;
  using my_al_ilqr::FiniteHorizonLQRSolver;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::Vector;

  // 本测试验证有限时域 LQR 求解器的核心正确性：
  // 1) Riccati 递推与反馈增益数量匹配时域长度；
  // 2) 闭环控制首个控制输入方向正确（对正位置偏差应施加负控制）；
  // 3) 终端状态接近参考点，说明稳定化生效；
  // 4) 增益非零，排除“未真正求解”的退化情况。
  const double dt = 0.1;

  Matrix A(2, 2);
  A << 1.0, dt,
       0.0, 1.0;
  Matrix B(2, 1);
  B << 0.5 * dt * dt,
       dt;

  Matrix Q = Matrix::Identity(2, 2);
  Matrix R = Matrix::Constant(1, 1, 0.2);
  Matrix Qf = 5.0 * Matrix::Identity(2, 2);

  Vector x_ref = Vector::Zero(2);
  Vector u_ref = Vector::Zero(1);

  FiniteHorizonLQRProblem problem{A, B, Q, R, Qf, x_ref, u_ref, 50};
  FiniteHorizonLQRSolver solver(problem);
  solver.Solve();

  Vector x0(2);
  x0 << 1.0, 0.0;
  const auto trajectory = solver.Simulate(x0);

  // 反馈矩阵和 Riccati 矩阵长度检查，验证离散最优控制递推完整执行。
  assert(static_cast<int>(solver.FeedbackGains().size()) == problem.horizon);
  assert(static_cast<int>(solver.RiccatiMatrices().size()) == problem.horizon + 1);

  // 仿真轨迹长度应与问题时域一致。
  assert(trajectory.Horizon() == problem.horizon);

  // 正位置偏差下首个最优控制应为负，体现“拉回原点”的反馈方向。
  assert(trajectory.Control(0)(0) < 0.0);

  // 终端状态应接近零，验证闭环收敛能力。
  assert(std::abs(trajectory.State(problem.horizon)(0)) < 1e-2);
  assert(std::abs(trajectory.State(problem.horizon)(1)) < 1e-2);

  // 增益范数非零，避免误把空解/零解当成正确解。
  assert(!NearlyEqual(solver.FeedbackGains().front().norm(), 0.0));

  return 0;
}
