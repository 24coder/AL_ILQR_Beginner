#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

// 终端目标等式约束。
// 常用于强制末状态满足 x_N = x_target。
// Evaluate 返回 (x - x_target)，因此满足约束时应为零向量。
class TerminalGoalConstraint final : public ConstraintFunction {
 public:
  explicit TerminalGoalConstraint(Vector target_state);

  ConstraintType Type() const override { return ConstraintType::kEquality; }
  int StateDim() const override { return target_state_.size(); }
  // 终端约束通常只依赖末状态，不依赖控制输入。
  int ControlDim() const override { return 0; }
  int OutputDim() const override { return target_state_.size(); }
  std::string Name() const override { return "terminal_goal"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  Vector target_state_;  // 目标末状态 x_target
};

}  // namespace my_al_ilqr
