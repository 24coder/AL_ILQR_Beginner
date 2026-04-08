#include "dynamics/kinematic_bicycle_model.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

Vector KinematicBicycleModel::NextState(const Vector& state,
                                        const Vector& control,
                                        double dt) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("KinematicBicycleModel received an input with invalid dimensions.");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive.");
  }
  if (wheelbase_ <= 0.0) {
    throw std::invalid_argument("Wheelbase must be positive.");
  }

  // 状态与控制定义：
  // state = [x, y, yaw, v]
  // control = [a, delta]
  const double x = state(0);
  const double y = state(1);
  const double yaw = state(2);
  const double velocity = state(3);
  const double acceleration = control(0);
  const double steering = control(1);

  // 运动学自行车离散更新（显式欧拉）：
  // x_{k+1}   = x_k + dt * v * cos(yaw)
  // y_{k+1}   = y_k + dt * v * sin(yaw)
  // yaw_{k+1} = yaw_k + dt * v * tan(delta) / L
  // v_{k+1}   = v_k + dt * a
  // 几何含义：转角 delta 通过曲率 tan(delta)/L 影响航向变化率。
  Vector next_state(StateDim());
  next_state(0) = x + dt * velocity * std::cos(yaw);
  next_state(1) = y + dt * velocity * std::sin(yaw);
  next_state(2) = yaw + dt * velocity * std::tan(steering) / wheelbase_;
  next_state(3) = velocity + dt * acceleration;
  return next_state;
}

}  // namespace my_al_ilqr
