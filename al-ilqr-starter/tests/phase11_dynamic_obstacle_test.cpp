#include <cassert>
#include <cmath>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  // 本测试验证“动态障碍物”场景下 AL-iLQR 的安全性与可达性：
  // 1) 优化后 best_violation 相比初始轨迹下降；
  // 2) best_violation 与最小安全裕度满足阈值，说明未发生严重碰撞穿透；
  // 3) 终端 x 位置达到目标下界，保证避障同时仍能前进。
  const auto scenario = my_al_ilqr::CreateDynamicObstacleDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  // 外层日志非空，说明求解流程正常执行。
  assert(!result.outer_logs.empty());

  // 优化后约束违反应优于初始值。
  assert(result.best_violation < initial_violation);

  // 对动态障碍案例设置的可接受违反阈值。
  assert(result.best_violation <= 0.12);

  // 下面手工计算“车辆圆近似 vs 障碍物圆”的最小安全裕度：
  // margin = d^2 - (r_vehicle + r_obstacle)^2，
  // margin < 0 表示发生重叠（违反安全距离）。
  const auto vehicle_circle =
      my_al_ilqr::BuildSingleCircleVehicleApproximation(scenario.vehicle_config.body);
  double min_clearance_margin = 1e9;
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    const auto& obstacle = scenario.obstacle_centers[k];
    const auto state = result.trajectory.State(k);
    const double c = std::cos(state(2));
    const double s = std::sin(state(2));
    const double circle_x =
        state(0) + c * vehicle_circle.center_x_body - s * vehicle_circle.center_y_body;
    const double circle_y =
        state(1) + s * vehicle_circle.center_x_body + c * vehicle_circle.center_y_body;
    const double dx = circle_x - obstacle(0);
    const double dy = circle_y - obstacle(1);
    const double safe_distance = scenario.obstacle_radius + vehicle_circle.radius;
    const double margin = dx * dx + dy * dy - safe_distance * safe_distance;
    min_clearance_margin = std::min(min_clearance_margin, margin);
  }

  // 安全裕度允许极小负值（数值离散/近似误差），但不得超阈。
  assert(min_clearance_margin >= -0.12);

  // 可达性要求：车辆仍需沿主方向推进到指定位置。
  assert(result.trajectory.State(scenario.base_problem->Horizon())(0) >= 10.0);

  return 0;
}
