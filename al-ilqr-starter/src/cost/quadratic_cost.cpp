#include "cost/quadratic_cost.hpp"

#include <stdexcept>

namespace my_al_ilqr {

// 构造二次型代价对象，并进行基础维度检查。
// 数据约定：
// - nx = state_reference 维度；
// - nu = control_reference 维度；
// - Q/Qf 为 (nx, nx)，R 为 (nu, nu)。
QuadraticCost::QuadraticCost(Matrix Q,
                             Matrix R,
                             Matrix Qf,
                             Vector state_reference,
                             Vector control_reference)
    : Q_(std::move(Q)),
      R_(std::move(R)),
      Qf_(std::move(Qf)),
      state_reference_(std::move(state_reference)),
      control_reference_(std::move(control_reference)) {
  const int nx = state_reference_.size();
  const int nu = control_reference_.size();
  if (nx <= 0 || nu <= 0) {
    throw std::invalid_argument("QuadraticCost dimensions must be positive.");
  }
  if (Q_.rows() != nx || Q_.cols() != nx || Qf_.rows() != nx || Qf_.cols() != nx) {
    throw std::invalid_argument("State weight matrices have incompatible dimensions.");
  }
  if (R_.rows() != nu || R_.cols() != nu) {
    throw std::invalid_argument("Control weight matrix has incompatible dimensions.");
  }
}

// 计算阶段代价：
//   l(x,u) = 0.5 * (x-x_ref)^T Q (x-x_ref) + 0.5 * (u-u_ref)^T R (u-u_ref)
// 其中 0.5 系数使得在对称权重矩阵下求导时表达式更简洁。
double QuadraticCost::StageCost(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("StageCost received an input with invalid dimensions.");
  }

  // 先转成“相对参考点”的误差坐标，再计算二次型。
  const Vector dx = state - state_reference_;
  const Vector du = control - control_reference_;
  return 0.5 * dx.dot(Q_ * dx) + 0.5 * du.dot(R_ * du);
}

// 计算终端代价：
//   l_f(x) = 0.5 * (x-x_ref)^T Qf (x-x_ref)
double QuadraticCost::TerminalCost(const Vector& state) const {
  if (state.size() != StateDim()) {
    throw std::invalid_argument("TerminalCost received a state with invalid dimensions.");
  }

  const Vector dx = state - state_reference_;
  return 0.5 * dx.dot(Qf_ * dx);
}

}  // namespace my_al_ilqr
