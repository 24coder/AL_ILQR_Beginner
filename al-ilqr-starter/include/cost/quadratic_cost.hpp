#pragma once

#include "cost/cost_function.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

// 标准二次型代价：
// - 阶段代价: 0.5 * (x-x_ref)^T Q (x-x_ref) + 0.5 * (u-u_ref)^T R (u-u_ref)
// - 终端代价: 0.5 * (x-x_ref)^T Qf (x-x_ref)
//
// 维度约定：
// - x in R^{nx}, u in R^{nu}
// - Q, Qf in R^{nx x nx}
// - R in R^{nu x nu}
class QuadraticCost final : public CostFunction {
 public:
  // 构造时保存权重矩阵与参考点，并在实现文件中检查维度一致性。
  QuadraticCost(Matrix Q, Matrix R, Matrix Qf, Vector state_reference, Vector control_reference);

  // 状态维度由参考状态向量长度给出。
  int StateDim() const override { return state_reference_.size(); }

  // 控制维度由参考控制向量长度给出。
  int ControlDim() const override { return control_reference_.size(); }

  // 计算单步阶段代价 l_k(x_k, u_k)。
  double StageCost(const Vector& state, const Vector& control) const override;

  // 计算终端代价 l_f(x_N)。
  double TerminalCost(const Vector& state) const override;

 private:
  // 阶段状态二次权重。
  Matrix Q_;

  // 阶段控制二次权重。
  Matrix R_;

  // 终端状态二次权重。
  Matrix Qf_;

  // 状态参考点 x_ref。
  Vector state_reference_;

  // 控制参考点 u_ref。
  Vector control_reference_;
};

}  // namespace my_al_ilqr
