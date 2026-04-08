#pragma once

#include <vector>

#include "core/trajectory.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

// 有限时域离散 LQR 问题定义：
//   x_{k+1} = A x_k + B u_k
// 代价函数：
//   sum_{k=0}^{N-1} [0.5(x_k-x_ref)^T Q (x_k-x_ref) + 0.5(u_k-u_ref)^T R (u_k-u_ref)]
//   + 0.5(x_N-x_ref)^T Qf (x_N-x_ref)
struct FiniteHorizonLQRProblem {
  Matrix A;
  Matrix B;
  Matrix Q;
  Matrix R;
  Matrix Qf;
  Vector state_reference;
  Vector control_reference;
  int horizon = 0;

  // nx = 状态维度，直接由 A 的行数给出。
  int StateDim() const { return A.rows(); }

  // nu = 控制维度，直接由 B 的列数给出。
  int ControlDim() const { return B.cols(); }
};

// 有限时域 LQR 求解器。
// Solve() 后可获取每个时刻的反馈增益 K_k，并用于闭环控制或模拟。
class FiniteHorizonLQRSolver {
 public:
  explicit FiniteHorizonLQRSolver(FiniteHorizonLQRProblem problem);

  // 执行 Riccati 反向递推，计算全时域反馈增益。
  void Solve();

  // 返回各时刻反馈增益 K_k（k=0...N-1，维度均为 nu x nx）。
  const std::vector<Matrix>& FeedbackGains() const { return feedback_gains_; }

  // 返回 Riccati 矩阵 P_k（k=0...N，维度均为 nx x nx）。
  const std::vector<Matrix>& RiccatiMatrices() const { return riccati_matrices_; }

  // 返回问题定义。
  const FiniteHorizonLQRProblem& Problem() const { return problem_; }

  // 查询第 k 时刻控制律：u_k = u_ref + K_k (x_k - x_ref)。
  Vector Control(const Vector& state, int k) const;

  // 从给定初始状态出发，用闭环控制律模拟一条轨迹。
  Trajectory Simulate(const Vector& initial_state) const;

 private:
  // 检查矩阵/向量/时域尺寸一致性。
  void ValidateProblem() const;

  // 问题数据副本。
  FiniteHorizonLQRProblem problem_;

  // 反馈增益缓存（长度 N）。
  std::vector<Matrix> feedback_gains_;

  // Riccati 矩阵缓存（长度 N+1）。
  std::vector<Matrix> riccati_matrices_;

  // 是否已完成 Solve()。
  bool is_solved_ = false;
};

}  // namespace my_al_ilqr
