#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

// 速度上下界不等式约束（针对状态中的速度分量 state(3)）。
// 形式：
// 1) min_speed - v <= 0
// 2) v - max_speed <= 0
// 两个分量均 <= 0 时即满足速度范围限制。
class SpeedLimitConstraint final : public ConstraintFunction {
 public:
  SpeedLimitConstraint(double min_speed, double max_speed);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 2; }
  std::string Name() const override { return "speed_limit"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  double min_speed_ = 0.0;
  double max_speed_ = 0.0;
};

}  // namespace my_al_ilqr
