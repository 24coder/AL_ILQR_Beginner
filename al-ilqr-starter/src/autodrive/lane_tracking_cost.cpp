#include "autodrive/lane_tracking_cost.hpp"

#include <stdexcept>

namespace my_al_ilqr {

LaneTrackingCost::LaneTrackingCost(std::shared_ptr<StraightReferenceLine> reference_line,
                                   double target_speed,
                                   double target_longitudinal_position,
                                   double stage_lateral_weight,
                                   double stage_heading_weight,
                                   double stage_speed_weight,
                                   double stage_accel_weight,
                                   double stage_steering_weight,
                                   double terminal_longitudinal_weight,
                                   double terminal_lateral_weight,
                                   double terminal_heading_weight,
                                   double terminal_speed_weight)
    : reference_line_(std::move(reference_line)),
      target_speed_(target_speed),
      target_longitudinal_position_(target_longitudinal_position),
      stage_lateral_weight_(stage_lateral_weight),
      stage_heading_weight_(stage_heading_weight),
      stage_speed_weight_(stage_speed_weight),
      stage_accel_weight_(stage_accel_weight),
      stage_steering_weight_(stage_steering_weight),
      terminal_longitudinal_weight_(terminal_longitudinal_weight),
      terminal_lateral_weight_(terminal_lateral_weight),
      terminal_heading_weight_(terminal_heading_weight),
      terminal_speed_weight_(terminal_speed_weight) {
  // 依赖注入的参考线用于统一计算几何误差（纵向/横向/航向）。
  if (!reference_line_) {
    throw std::invalid_argument("LaneTrackingCost requires a valid reference line.");
  }
}

double LaneTrackingCost::StageCost(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("LaneTrackingCost::StageCost received an input with invalid dimensions.");
  }

  // 几何/运动误差提取：
  // - lateral_error: 到参考线法向偏差
  // - heading_error: 航向相对参考方向偏差
  // - speed_error: 速度相对目标速度偏差
  const double lateral_error = reference_line_->LateralError(state);
  const double heading_error = reference_line_->HeadingError(state);
  const double speed_error = state(3) - target_speed_;

  // 二次型阶段代价：0.5 * w * e^2。
  // 同时惩罚控制量幅值，抑制过激加速度/转角输入。
  return 0.5 * stage_lateral_weight_ * lateral_error * lateral_error +
         0.5 * stage_heading_weight_ * heading_error * heading_error +
         0.5 * stage_speed_weight_ * speed_error * speed_error +
         0.5 * stage_accel_weight_ * control(0) * control(0) +
         0.5 * stage_steering_weight_ * control(1) * control(1);
}

double LaneTrackingCost::TerminalCost(const Vector& state) const {
  if (state.size() != StateDim()) {
    throw std::invalid_argument(
        "LaneTrackingCost::TerminalCost received a state with invalid dimensions.");
  }

  // 终端误差：
  // longitudinal_error 推动车辆在终点达到目标纵向里程；
  // lateral/heading/speed 约束终点姿态与速度质量。
  const double longitudinal_error =
      reference_line_->LongitudinalPosition(state) - target_longitudinal_position_;
  const double lateral_error = reference_line_->LateralError(state);
  const double heading_error = reference_line_->HeadingError(state);
  const double speed_error = state(3) - target_speed_;

  return 0.5 * terminal_longitudinal_weight_ * longitudinal_error * longitudinal_error +
         0.5 * terminal_lateral_weight_ * lateral_error * lateral_error +
         0.5 * terminal_heading_weight_ * heading_error * heading_error +
         0.5 * terminal_speed_weight_ * speed_error * speed_error;
}

}  // namespace my_al_ilqr
