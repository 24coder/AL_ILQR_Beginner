#include "autodrive/speed_limit_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

SpeedLimitConstraint::SpeedLimitConstraint(double min_speed, double max_speed)
    : min_speed_(min_speed), max_speed_(max_speed) {
  if (min_speed_ > max_speed_) {
    throw std::invalid_argument("SpeedLimitConstraint bounds are inconsistent.");
  }
}

Vector SpeedLimitConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("SpeedLimitConstraint received an input with invalid dimensions.");
  }

  // 对速度分量 v = state(3) 构造双边不等式：
  // min_speed - v <= 0,  v - max_speed <= 0。
  Vector values(OutputDim());
  values(0) = min_speed_ - state(3);
  values(1) = state(3) - max_speed_;
  return values;
}

}  // namespace my_al_ilqr
