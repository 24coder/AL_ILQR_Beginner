#pragma once

#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

// 独轮车模型（Unicycle）。
// 状态 x = [px, py, heading]，控制 u = [v, yaw_rate]。
// 适合平面位姿与转向速率控制的基础验证场景。
class UnicycleModel final : public DynamicsModel {
 public:
  int StateDim() const override { return 3; }
  int ControlDim() const override { return 2; }
  Vector NextState(const Vector& state, const Vector& control, double dt) const override;
};

}  // namespace my_al_ilqr
