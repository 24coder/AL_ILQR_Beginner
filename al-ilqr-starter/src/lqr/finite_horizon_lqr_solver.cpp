#include "lqr/finite_horizon_lqr_solver.hpp"

#include <stdexcept>

namespace my_al_ilqr {

// 构造有限时域 LQR 求解器。
// 这里把 problem 按值传入再 move 到成员变量，意味着求解器内部持有自己的一份问题定义。
// 构造时立即完成两件事：
// 1. 检查问题定义在维度上是否合法；
// 2. 预分配反馈增益和 Riccati 矩阵缓存，避免 Solve() 时重复按尺寸构造。
FiniteHorizonLQRSolver::FiniteHorizonLQRSolver(FiniteHorizonLQRProblem problem)
    : problem_(std::move(problem)) {
  ValidateProblem();
  feedback_gains_.assign(problem_.horizon, Matrix::Zero(problem_.ControlDim(), problem_.StateDim()));
  riccati_matrices_.assign(problem_.horizon + 1,
                           Matrix::Zero(problem_.StateDim(), problem_.StateDim()));
}

// 检查有限时域 LQR 问题的基本合法性。
// 这一层只检查“尺寸是否匹配”，不检查更强的数学条件，诸如：
// - Q / Qf 是否半正定；
// - R 是否正定；
// - 系统是否可控。
// 这些条件若不满足，后续求解虽然可能仍能运行，但控制律的数值性质不一定可靠。
void FiniteHorizonLQRSolver::ValidateProblem() const {
  if (problem_.horizon <= 0) {
    throw std::invalid_argument("LQR horizon must be positive.");
  }
  if (problem_.A.rows() == 0 || problem_.A.cols() == 0 || problem_.B.rows() == 0 ||
      problem_.B.cols() == 0) {
    throw std::invalid_argument("LQR system matrices must be non-empty.");
  }

  // 对离散线性系统 x_{k+1} = A x_k + B u_k，A 必须是方阵，
  // 因为它表示“状态到下一状态”的线性映射，输入和输出都在状态空间中。
  if (problem_.A.rows() != problem_.A.cols()) {
    throw std::invalid_argument("LQR matrix A must be square.");
  }

  // B 的行数必须与 A 的行数一致，
  // 因为 A x_k 和 B u_k 需要能相加，二者都必须落在同一个状态空间维度 nx 中。
  if (problem_.B.rows() != problem_.A.rows()) {
    throw std::invalid_argument("LQR matrix B must have the same number of rows as A.");
  }

  const int nx = problem_.StateDim();
  const int nu = problem_.ControlDim();

  // Q 和 Qf 都是状态二次代价矩阵，因此必须是 (nx, nx)。
  // 其中：
  // - Q 作用于中间各时刻；
  // - Qf 作用于终端状态。
  if (problem_.Q.rows() != nx || problem_.Q.cols() != nx || problem_.Qf.rows() != nx ||
      problem_.Qf.cols() != nx) {
    throw std::invalid_argument("LQR state cost matrices have incompatible dimensions.");
  }

  // R 是控制二次代价矩阵，因此必须是 (nu, nu)。
  if (problem_.R.rows() != nu || problem_.R.cols() != nu) {
    throw std::invalid_argument("LQR control cost matrix has incompatible dimensions.");
  }

  // 参考状态和参考控制分别用于定义误差：
  //   x_error = x - x_ref
  //   u_error = u - u_ref
  // 因此它们的长度必须分别匹配状态维度和控制维度。
  if (problem_.state_reference.size() != nx || problem_.control_reference.size() != nu) {
    throw std::invalid_argument("LQR reference vectors have incompatible dimensions.");
  }
}

// 求解有限时域 LQR。
// 这里实现的是标准离散时间、有限时域 Riccati 反向递推：
//
//   P_N = Qf
//   K_k = -(R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A
//   P_k = Q + A^T P_{k+1} (A + B K_k)
//
// 求解完成后，会得到每个时刻对应的最优线性反馈增益 K_k。
// 这些 K_k 之后会被 Control() 和 Simulate() 使用。
void FiniteHorizonLQRSolver::Solve() {
  // 终端 Riccati 矩阵就是终端状态代价矩阵。
  // 这对应动态规划的边界条件：终端 cost-to-go 只剩下终端代价。
  riccati_matrices_.back() = problem_.Qf;

  // 从最后一个控制时刻开始，向前递推到 k = 0。
  for (int k = problem_.horizon - 1; k >= 0; --k) {
    const Matrix& P_next = riccati_matrices_[k + 1];

    // S 是当前时刻对控制变量的二次型系数：
    //   S = R + B^T P_{k+1} B
    // 若从直觉上看：
    // - R 惩罚“控制本身过大”；
    // - B^T P_next B 惩罚“控制通过状态演化对未来代价带来的影响”。
    const Matrix S = problem_.R + problem_.B.transpose() * P_next * problem_.B;

    // rhs 对应最优反馈律中的右端项：B^T P_{k+1} A。
    const Matrix rhs = problem_.B.transpose() * P_next * problem_.A;

    // 解出最优反馈增益 K_k。
    // 这里使用 LDLT 分解求解线性方程，而不是显式求逆：
    // - 数值上更稳定；
    // - 计算上更高效。
    feedback_gains_[k] = -S.ldlt().solve(rhs);

    // 递推当前时刻的 Riccati 矩阵 P_k。
    // P_k 可以理解为“从时刻 k 开始，状态误差对最优剩余代价的二次近似权重”。
    riccati_matrices_[k] =
        problem_.Q + problem_.A.transpose() * P_next * (problem_.A + problem_.B * feedback_gains_[k]);
  }

  is_solved_ = true;
}

// 查询第 k 个时刻的控制律。
// 这里使用的是围绕参考点的线性状态反馈：
//   u_k = u_ref + K_k (x_k - x_ref)
// 其中：
// - x_ref 是希望跟踪的参考状态；
// - u_ref 是对应的参考控制；
// - K_k 是 Solve() 中反向递推得到的最优反馈增益。
Vector FiniteHorizonLQRSolver::Control(const Vector& state, int k) const {
  if (!is_solved_) {
    throw std::logic_error("LQR solver must be solved before querying controls.");
  }
  if (k < 0 || k >= problem_.horizon) {
    throw std::out_of_range("LQR control index is out of range.");
  }
  if (state.size() != problem_.StateDim()) {
    throw std::invalid_argument("LQR control query received a state with invalid dimensions.");
  }

  // 先计算当前状态相对参考状态的偏差。
  // 反馈项真正作用的不是绝对状态，而是“离参考点偏了多少”。
  const Vector state_error = state - problem_.state_reference;

  // 在参考控制的基础上，加上由状态偏差触发的反馈修正。
  return problem_.control_reference + feedback_gains_[k] * state_error;
}

// 使用已求出的时变反馈律，从给定初始状态出发模拟整条闭环轨迹。
// 数据流是：
//   x_0
//    -> u_0 = Control(x_0, 0)
//    -> x_1 = A x_0 + B u_0
//    -> u_1 = Control(x_1, 1)
//    -> x_2 = A x_1 + B u_1
//    -> ...
// 直到时域末端。
Trajectory FiniteHorizonLQRSolver::Simulate(const Vector& initial_state) const {
  if (!is_solved_) {
    throw std::logic_error("LQR solver must be solved before simulation.");
  }
  if (initial_state.size() != problem_.StateDim()) {
    throw std::invalid_argument("LQR simulation received an initial state with invalid dimensions.");
  }

  // 创建长度为 horizon 的轨迹对象。
  // 按该项目 Trajectory 的约定：
  // - 有 horizon 个控制点 u_0 ... u_{N-1}；
  // - 有 horizon + 1 个状态点 x_0 ... x_N。
  Trajectory trajectory(problem_.StateDim(), problem_.ControlDim(), problem_.horizon);
  trajectory.State(0) = initial_state;

  for (int k = 0; k < problem_.horizon; ++k) {
    // 先根据当前状态和当前时刻的反馈增益求控制。
    trajectory.Control(k) = Control(trajectory.State(k), k);

    // 再用离散线性动力学推进到下一时刻。
    trajectory.State(k + 1) = problem_.A * trajectory.State(k) + problem_.B * trajectory.Control(k);
  }

  return trajectory;
}

}  // namespace my_al_ilqr
