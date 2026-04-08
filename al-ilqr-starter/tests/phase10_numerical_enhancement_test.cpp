#include <cassert>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  // 本测试验证在“更激进的数值参数”下 AL-iLQR 的鲁棒性与返回语义：
  // 1) 外层日志存在，说明迭代过程完整；
  // 2) best_violation 应不大于 final_violation（最佳值定义一致）；
  // 3) best_violation_so_far 单调不增（历史最优应不断保持或改进）；
  // 4) 至少发生一次罚因子更新；
  // 5) 返回轨迹与 best_violation 对齐（当 return_best_trajectory=true）。
  auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();
  scenario.solver_options.max_outer_iterations = 10;
  scenario.solver_options.constraint_tolerance = 1e-2;
  scenario.solver_options.initial_penalty = 2.0;
  scenario.solver_options.penalty_scaling = 4.0;
  scenario.solver_options.penalty_update_ratio = 0.92;
  scenario.solver_options.max_penalty = 1e8;
  scenario.solver_options.return_best_trajectory = true;

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  // 外层必须执行。
  assert(!result.outer_logs.empty());

  // 最优违反量定义检查。
  assert(result.best_violation <= result.final_violation + 1e-12);

  bool has_penalty_update = false;
  double previous_best = result.outer_logs.front().best_violation_so_far;
  for (const auto& log : result.outer_logs) {
    // 历史最优违反量应单调不增。
    assert(log.best_violation_so_far <= previous_best + 1e-12);
    previous_best = log.best_violation_so_far;
    has_penalty_update = has_penalty_update || log.penalty_updated;
  }

  // 数值增强参数下应出现至少一次罚因子更新。
  assert(has_penalty_update);

  // 返回轨迹应对应 best_violation（而非仅最后一次迭代）。
  const double returned_violation = scenario.constrained_problem->MaxViolation(result.trajectory);
  assert(std::abs(returned_violation - result.best_violation) <= 1e-10);

  return 0;
}
