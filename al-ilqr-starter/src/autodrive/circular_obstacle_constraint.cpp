#include "autodrive/circular_obstacle_constraint.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

CircularObstacleConstraint::CircularObstacleConstraint(double obstacle_center_x,
                                                       double obstacle_center_y,
                                                       double obstacle_radius,
                                                       VehicleCollisionCircle vehicle_circle)
    : obstacle_center_x_(obstacle_center_x),
      obstacle_center_y_(obstacle_center_y),
      obstacle_radius_(obstacle_radius),
      vehicle_circle_(vehicle_circle) {
  if (obstacle_radius_ <= 0.0) {
    throw std::invalid_argument("CircularObstacleConstraint obstacle_radius must be positive.");
  }
  if (vehicle_circle_.radius <= 0.0) {
    throw std::invalid_argument("CircularObstacleConstraint vehicle circle radius must be positive.");
  }
}

Vector CircularObstacleConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "CircularObstacleConstraint received an input with invalid dimensions.");
  }

  // 车辆圆心从车体坐标系旋转平移到世界坐标系：
  // p_world = p_rear_axle + R(yaw) * p_body
  const double c = std::cos(state(2));
  const double s = std::sin(state(2));
  const double vehicle_center_x =
      state(0) + c * vehicle_circle_.center_x_body - s * vehicle_circle_.center_y_body;
  const double vehicle_center_y =
      state(1) + s * vehicle_circle_.center_x_body + c * vehicle_circle_.center_y_body;

  // 圆-圆安全距离约束：
  // ||p_vehicle - p_obstacle||^2 >= (r_vehicle + r_obstacle)^2
  // 等价写成不等式 h<=0：
  // h = (r_sum)^2 - distance^2 <= 0
  const double dx = vehicle_center_x - obstacle_center_x_;
  const double dy = vehicle_center_y - obstacle_center_y_;
  const double safe_radius = obstacle_radius_ + vehicle_circle_.radius;
  Vector values(OutputDim());
  values(0) = safe_radius * safe_radius - (dx * dx + dy * dy);
  return values;
}

}  // namespace my_al_ilqr
