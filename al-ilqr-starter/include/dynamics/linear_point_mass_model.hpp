#pragma once

#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

// 一维线性质点模型。
// 状态 x = [p, v]（位置、速度），控制 u = [a]（加速度）。
// 采用匀加速度离散更新，常用于最小示例或纵向控制原型。
class LinearPointMassModel final : public DynamicsModel {
 public:
  int StateDim() const override { return 2; }
  int ControlDim() const override { return 1; }
  Vector NextState(const Vector& state, const Vector& control, double dt) const override;
};

}  // namespace my_al_ilqr
