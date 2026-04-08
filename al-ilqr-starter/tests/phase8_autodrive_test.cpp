#include <cassert>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::Vector;

  // 本测试验证自动驾驶演示场景在默认参数下可稳定求解：
  // 1) 求解后约束违反量不应比初始轨迹更差；
  // 2) 违反量应控制在工程可接受范围；
  // 3) 终端纵向位置与速度达到最低目标，证明“可行且有效推进”。
  const auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  // 外层日志非空，说明 AL 外层流程确实运行。
  assert(!result.outer_logs.empty());

  const double returned_violation = scenario.constrained_problem->MaxViolation(result.trajectory);

  // 结果轨迹不应恶化违反量。
  assert(returned_violation <= initial_violation + 1e-9);

  // 违反量阈值约束：保证方案可用。
  assert(returned_violation <= 0.2);

  // 终端状态下界检查：车辆应前进到目标区间并保持基本速度。
  const Vector terminal_state = result.trajectory.State(scenario.base_problem->Horizon());
  assert(terminal_state(0) >= 10.5);
  assert(terminal_state(3) >= 2.0);

  return 0;
}
