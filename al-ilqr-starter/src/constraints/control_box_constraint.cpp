#include "constraints/control_box_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

ControlBoxConstraint::ControlBoxConstraint(int state_dim, Vector lower_bound, Vector upper_bound)
    : state_dim_(state_dim), lower_bound_(std::move(lower_bound)), upper_bound_(std::move(upper_bound)) {
  if (state_dim_ <= 0) {
    throw std::invalid_argument("ControlBoxConstraint state dimension must be positive.");
  }
  if (lower_bound_.size() == 0 || upper_bound_.size() == 0) {
    throw std::invalid_argument("ControlBoxConstraint bounds must be non-empty.");
  }
  if (lower_bound_.size() != upper_bound_.size()) {
    throw std::invalid_argument("ControlBoxConstraint bounds must have matching dimensions.");
  }
}

Vector ControlBoxConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (control.size() != ControlDim()) {
    throw std::invalid_argument("ControlBoxConstraint received a control with invalid dimensions.");
  }
  if (state.size() != state_dim_) {
    throw std::invalid_argument("ControlBoxConstraint received inconsistent state dimensions.");
  }

  // 输出向量按 [下界约束, 上界约束] 拼接：
  // 前 m 维：lower - u <= 0
  // 后 m 维：u - upper <= 0
  // 数据流：control -> 按元素与边界比较 -> 交给约束评估模块。
  Vector values(OutputDim());
  values.head(ControlDim()) = lower_bound_ - control;
  values.tail(ControlDim()) = control - upper_bound_;
  return values;
}

}  // namespace my_al_ilqr
