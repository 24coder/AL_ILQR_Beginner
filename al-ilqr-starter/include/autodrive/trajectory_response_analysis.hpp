#pragma once

#include <cmath>

#include "core/trajectory.hpp"

namespace my_al_ilqr {

// 轨迹行为类型：
// - kGoStraight: 近似直行通过
// - kYield: 通过时间明显推迟（礼让）
// - kBypass: 横向偏移显著（绕行）
enum class TrajectoryResponseType {
  kGoStraight,
  kYield,
  kBypass,
};

// 轨迹响应分析指标。
struct TrajectoryResponseMetrics {
  double crossing_time = 0.0;      // 到达 obstacle_x 的时间
  double crossing_delay = 0.0;     // 相对基线时间的延迟
  double max_abs_lateral = 0.0;    // 轨迹最大横向绝对偏移 |y|
  double min_speed = 0.0;          // 全程最小速度
  TrajectoryResponseType response = TrajectoryResponseType::kGoStraight;
};

// 计算轨迹在 x 方向首次达到 target_x 的时间。
// 若整个时域都未达到，则返回末时刻时间。
inline double CrossingTimeAtX(const Trajectory& trajectory, double target_x) {
  for (int k = 0; k <= trajectory.Horizon(); ++k) {
    if (trajectory.State(k)(0) >= target_x) {
      return k * trajectory.TimeStep();
    }
  }
  return trajectory.Horizon() * trajectory.TimeStep();
}

// 按规则对轨迹行为进行分类。
// 数据流：Trajectory -> 统计指标(时延/横向偏移/最低速) -> response 标签。
inline TrajectoryResponseMetrics AnalyzeTrajectoryResponse(const Trajectory& trajectory,
                                                          double obstacle_x,
                                                          double baseline_crossing_time,
                                                          double bypass_lateral_threshold = 0.15,
                                                          double yield_delay_threshold = 0.35) {
  TrajectoryResponseMetrics metrics;
  metrics.crossing_time = CrossingTimeAtX(trajectory, obstacle_x);
  metrics.crossing_delay = metrics.crossing_time - baseline_crossing_time;
  metrics.min_speed = trajectory.State(0)(3);

  for (int k = 0; k <= trajectory.Horizon(); ++k) {
    metrics.max_abs_lateral = std::max(metrics.max_abs_lateral, std::abs(trajectory.State(k)(1)));
    metrics.min_speed = std::min(metrics.min_speed, trajectory.State(k)(3));
  }

  if (metrics.max_abs_lateral >= bypass_lateral_threshold) {
    metrics.response = TrajectoryResponseType::kBypass;
  } else if (metrics.crossing_delay >= yield_delay_threshold) {
    metrics.response = TrajectoryResponseType::kYield;
  } else {
    metrics.response = TrajectoryResponseType::kGoStraight;
  }
  return metrics;
}

inline const char* ToString(TrajectoryResponseType response) {
  switch (response) {
    case TrajectoryResponseType::kGoStraight:
      return "go_straight";
    case TrajectoryResponseType::kYield:
      return "yield";
    case TrajectoryResponseType::kBypass:
      return "bypass";
  }
  return "unknown";
}

}  // namespace my_al_ilqr
