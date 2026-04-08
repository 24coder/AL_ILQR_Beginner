#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

// 控制量盒约束（Box Constraint）。
// 对每个控制分量 u_i 施加区间限制：lower_i <= u_i <= upper_i。
// 在不等式标准形式 h(x,u)<=0 下，会拆成两组：
// 1) lower - u <= 0
// 2) u - upper <= 0
class ControlBoxConstraint final : public ConstraintFunction {
 public:
  // state_dim 仅用于输入维度一致性检查；
  // lower/upper 的长度定义了控制维度。
  ControlBoxConstraint(int state_dim, Vector lower_bound, Vector upper_bound);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return state_dim_; }
  int ControlDim() const override { return lower_bound_.size(); }
  // 输出维度是 2m：每个控制维度对应上下界两个不等式。
  int OutputDim() const override { return 2 * lower_bound_.size(); }
  std::string Name() const override { return "control_box"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  int state_dim_ = 0;
  Vector lower_bound_;  // 控制下界向量
  Vector upper_bound_;  // 控制上界向量
};

}  // namespace my_al_ilqr
