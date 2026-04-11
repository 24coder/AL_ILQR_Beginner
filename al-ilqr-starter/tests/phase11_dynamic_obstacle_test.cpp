#include <cassert>
#include <cmath>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  // 本测试验证“动态障碍物”场景下 AL-iLQR 的安全性、可达性，
  // 以及绕障后能够重新回归中心线：
  // 1) 优化后 best_violation 相比初始轨迹下降；
  // 2) best_violation 与最小安全裕度满足阈值，说明未发生严重碰撞穿透；
  // 3) 终端 x 位置达到目标下界，保证避障同时仍能前进；
  // 4) 终端横向误差与航向误差足够小，说明车辆已重新贴近中心线。
  const auto scenario = my_al_ilqr::CreateDynamicObstacleDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  std::cout << "Initial trajectory\n";
  initial_trajectory.Print();
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);
  std::cout << "AL-iLQR Result:trajectory\n";
  result.trajectory.Print();
  // 外层日志非空，说明求解流程正常执行。
  assert(!result.outer_logs.empty());

  // 若初始轨迹本身已可行，则至少不能显著恶化；
  // 若初始轨迹不可行，则优化后必须优于初始值。
  if (initial_violation > 1e-9) {
    assert(result.best_violation < initial_violation);
  } else {
    assert(result.best_violation <= 0.12);
  }

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

  const auto terminal_state = result.trajectory.State(scenario.base_problem->Horizon());
  const double terminal_lateral_error = scenario.reference_line->LateralError(terminal_state);
  const double terminal_heading_error = scenario.reference_line->HeadingError(terminal_state);

  // 可达性要求：车辆仍需沿主方向推进到指定位置。
  assert(terminal_state(0) >= 14.0);

  // 绕障后应重新回到中心线附近，而不是停留在较大横向偏移处。
  assert(std::abs(terminal_lateral_error) <= 3.3);
  assert(std::abs(terminal_heading_error) <= 0.35);

  return 0;
}
