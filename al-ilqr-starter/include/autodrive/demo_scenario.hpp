#pragma once

#include <memory>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/straight_reference_line.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "core/types.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

// 动态障碍场景参数。
// 障碍物沿 y 方向做匀速运动：y(t) = obstacle_initial_y + obstacle_speed_y * t。
struct DynamicObstacleScenarioConfig {
  double obstacle_center_x = 6.0;
  double obstacle_initial_y = -2.0;
  double obstacle_speed_y = 1.0;
  double obstacle_radius = 0.9;
  double road_half_width = 1.5;
  double stage_lateral_weight = 3.0;
  double stage_heading_weight = 0.5;
  double stage_speed_weight = 0.5;
  double stage_accel_weight = 0.2;
  double stage_steering_weight = 0.4;
  double terminal_longitudinal_weight = 20.0;
  double terminal_lateral_weight = 30.0;
  double terminal_heading_weight = 8.0;
  double terminal_speed_weight = 8.0;
  double initial_accel_guess = 0.15;
  int accel_guess_steps = 10;
  double initial_steering_guess = 0.0;
};

// 静态障碍场景参数（障碍物中心固定）。
struct StaticObstacleScenarioConfig {
  double obstacle_center_x = 6.0;
  double obstacle_center_y = 0.0;
  double obstacle_radius = 0.8;
  double road_half_width = 2.3;
  double initial_x = 0.0;
  double initial_y = 0.0;
  double initial_yaw = 0.0;
  double initial_speed = 2.5;
  double stage_lateral_weight = 1.0;
  double stage_heading_weight = 0.5;
  double stage_speed_weight = 0.5;
  double stage_accel_weight = 0.2;
  double stage_steering_weight = 0.4;
  double terminal_longitudinal_weight = 20.0;
  double terminal_lateral_weight = 12.0;
  double terminal_heading_weight = 6.0;
  double terminal_speed_weight = 8.0;
};

// 自动驾驶演示问题装配结果。
// 数据流：
// 配置 -> reference_line/dynamics/cost/problem -> 约束装配 -> 求解初值与求解器参数。
struct AutodriveDemoScenario {
  std::shared_ptr<StraightReferenceLine> reference_line;
  std::shared_ptr<OptimalControlProblem> base_problem;
  std::shared_ptr<ConstrainedOptimalControlProblem> constrained_problem;
  VehicleBicycleConfig vehicle_config;
  VehicleCollisionCircle vehicle_collision_circle;
  Vector initial_state;
  std::vector<Vector> initial_controls;
  ALILQROptions solver_options;
  double road_lower_bound = 0.0;
  double road_upper_bound = 0.0;
  double obstacle_center_x = 0.0;
  double obstacle_center_y = 0.0;
  double obstacle_radius = 0.0;
  std::vector<Vector> obstacle_centers;  // 每个阶段预测障碍中心（至少 Horizon+1 个）
  double vehicle_length = 0.0;
  double vehicle_width = 0.0;
  double rear_axle_to_rear = 0.0;
};

AutodriveDemoScenario CreateAutodriveDemoScenario();
AutodriveDemoScenario CreateSingleStaticObstacleTestScenario();
AutodriveDemoScenario CreateSingleStaticObstacleTestScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& config);
AutodriveDemoScenario CreateDynamicObstacleDemoScenario();
AutodriveDemoScenario CreateDynamicObstacleDemoScenario(const DynamicObstacleScenarioConfig& config);
AutodriveDemoScenario CreateDynamicObstacleSpeedStudyScenario(double obstacle_speed_y);

}  // namespace my_al_ilqr
