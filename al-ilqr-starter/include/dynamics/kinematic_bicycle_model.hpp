#pragma once

#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

// 运动学自行车模型（Kinematic Bicycle）。
// 状态 x = [px, py, yaw, v]，控制 u = [a, delta]。
// 其中 delta 为前轮转角，yaw 变化率近似为 v * tan(delta) / L。
class KinematicBicycleModel final : public DynamicsModel {
 public:
  // wheelbase: 轴距 L（前后轴中心距离），必须为正。
  explicit KinematicBicycleModel(double wheelbase = 2.8) : wheelbase_(wheelbase) {}

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double Wheelbase() const { return wheelbase_; }

  // 离散一步积分（显式欧拉）：x_{k+1} = f(x_k, u_k, dt)。
  Vector NextState(const Vector& state, const Vector& control, double dt) const override;

 private:
  double wheelbase_;
};

}  // namespace my_al_ilqr
