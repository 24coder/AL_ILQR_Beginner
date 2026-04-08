#include <filesystem>
#include <iostream>

#include "core/types.hpp"
#include "lqr/finite_horizon_lqr_solver.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  // Phase 2：有限时域线性 LQR。
  // 目的：展示在线性系统下，解析/递推结构得到的最优反馈如何驱动状态收敛。
  using my_al_ilqr::FiniteHorizonLQRProblem;
  using my_al_ilqr::FiniteHorizonLQRSolver;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::Vector;

  const double dt = 0.1;

  // 双积分线性系统：x=[position, velocity], u=[accel]
  Matrix A(2, 2);
  A << 1.0, dt,
       0.0, 1.0;

  Matrix B(2, 1);
  B << 0.5 * dt * dt,
       dt;

  // 关键权重：Qf(终端)显著大于 Q(过程)，体现“末端精度优先”。
  Matrix Q = Matrix::Zero(2, 2);
  Q.diagonal() << 2.0, 0.2;
  Matrix R = Matrix::Constant(1, 1, 0.1);
  Matrix Qf = Matrix::Zero(2, 2);
  Qf.diagonal() << 20.0, 1.0;

  Vector x_ref(2);
  x_ref << 1.5, 0.0;
  Vector u_ref = Vector::Zero(1);

  FiniteHorizonLQRProblem problem{A, B, Q, R, Qf, x_ref, u_ref, 60};
  FiniteHorizonLQRSolver solver(problem);
  // 执行流：先 Solve 计算反馈律，再 Simulate 前向仿真。
  solver.Solve();

  Vector x0 = Vector::Zero(2);
  const auto trajectory = solver.Simulate(x0);

  const std::filesystem::path csv_path = "build/phase2_linear_lqr_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, trajectory);

  std::cout << "Phase 2 linear LQR demo\n";
  std::cout << "horizon = " << problem.horizon << ", dt = " << dt << "\n";
  std::cout << "x[0] = " << trajectory.State(0).transpose() << "\n";
  std::cout << "x[10] = " << trajectory.State(10).transpose() << "\n";
  std::cout << "x[30] = " << trajectory.State(30).transpose() << "\n";
  std::cout << "x[60] = " << trajectory.State(60).transpose() << "\n";
  std::cout << "u[0] = " << trajectory.Control(0).transpose() << "\n";
  std::cout << "u[10] = " << trajectory.Control(10).transpose() << "\n";
  std::cout << "trajectory csv = " << csv_path << "\n";

  return 0;
}
