#pragma once

#include <filesystem>

namespace my_al_ilqr {

// 自行车运动学参数配置。
// 这些参数直接影响车辆状态更新与控制约束范围。
struct BicycleKinematicConfig {
  double wheelbase = 2.8;         // 轴距 L
  double min_acceleration = -1.0; // 最小纵向加速度 [m/s^2]
  double max_acceleration = 1.0;  // 最大纵向加速度 [m/s^2]
  double min_steering = -0.45;    // 最小前轮转角 [rad]
  double max_steering = 0.45;     // 最大前轮转角 [rad]
  double min_speed = 0.0;         // 最低车速 [m/s]
  double max_speed = 6.0;         // 最高车速 [m/s]
};

// 车辆几何尺寸配置（矩形包络）。
struct VehicleBodyConfig {
  double length = 4.6;            // 车长（前后最外轮廓距离）
  double width = 1.9;             // 车宽
  double rear_axle_to_rear = 1.0; // 后轴中心到车尾距离
};

// 碰撞检测使用的单圆近似参数。
// center_x_body / center_y_body 在车体坐标系下定义。
struct VehicleCollisionCircle {
  double center_x_body = 0.0;
  double center_y_body = 0.0;
  double radius = 0.0;
};

// 完整车辆配置：运动学 + 车体几何。
struct VehicleBicycleConfig {
  BicycleKinematicConfig model;
  VehicleBodyConfig body;
};

// 返回默认 ini 配置文件路径（源码目录下 config/vehicle_bicycle_config.ini）。
std::filesystem::path DefaultVehicleBicycleConfigPath();
// 从 ini 文件加载配置并进行合法性校验。
VehicleBicycleConfig LoadVehicleBicycleConfig(const std::filesystem::path& path);
// 将矩形车体构造为单圆近似，用于快速碰撞约束。
VehicleCollisionCircle BuildSingleCircleVehicleApproximation(const VehicleBodyConfig& config);

}  // namespace my_al_ilqr
