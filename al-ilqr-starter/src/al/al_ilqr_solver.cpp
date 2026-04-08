#include "al/al_ilqr_solver.hpp"

#include <stdexcept>
#include <iostream>

namespace my_al_ilqr {

namespace {

// 从轨迹中提取控制序列 u_0...u_{N-1}，作为下一轮内层 iLQR 的初始化。
std::vector<Vector> ExtractControls(const Trajectory& trajectory) {
  std::vector<Vector> controls(trajectory.Horizon(), Vector::Zero(trajectory.ControlDim()));
  for (int k = 0; k < trajectory.Horizon(); ++k) {
    controls[k] = trajectory.Control(k);
  }
  return controls;
}

}  // namespace

// 构造 AL-iLQR 求解器并检查外层参数有效性。
ALILQRSolver::ALILQRSolver(const ConstrainedOptimalControlProblem& problem, ALILQROptions options)
    : problem_(problem), options_(std::move(options)) {
  if (options_.max_outer_iterations <= 0) {
    throw std::invalid_argument("AL-iLQR max_outer_iterations must be positive.");
  }
  if (options_.constraint_tolerance <= 0.0) {
    throw std::invalid_argument("AL-iLQR constraint_tolerance must be positive.");
  }
  if (options_.initial_penalty <= 0.0 || options_.penalty_scaling <= 1.0) {
    throw std::invalid_argument("AL-iLQR penalty parameters must satisfy mu > 0 and scaling > 1.");
  }
  if (options_.penalty_update_ratio <= 0.0 || options_.penalty_update_ratio >= 1.0) {
    throw std::invalid_argument("AL-iLQR penalty_update_ratio must lie in (0, 1).");
  }
  if (options_.max_penalty <= 0.0) {
    throw std::invalid_argument("AL-iLQR max_penalty must be positive.");
  }
}

// AL-iLQR 主流程：
// 外层（AL）循环负责处理约束，内层（iLQR）负责优化当前增广无约束子问题。
ALILQRResult ALILQRSolver::Solve(const Vector& initial_state,
                                 const std::vector<Vector>& initial_controls) {
  if (initial_state.size() != problem_.BaseProblem().StateDim()) {
    throw std::invalid_argument("AL-iLQR received an initial state with invalid dimensions.");
  }
  if (static_cast<int>(initial_controls.size()) != problem_.Horizon()) {
    throw std::invalid_argument("AL-iLQR received a control sequence with invalid length.");
  }

  outer_log_.clear();

  // 构造增广子问题，并用用户给定控制序列初始化当前轨迹。
  AugmentedLagrangianProblem al_problem(problem_, options_.initial_penalty, options_.penalty_scaling);
  std::vector<Vector> controls = initial_controls;
  Trajectory current_trajectory = problem_.BaseProblem().Rollout(initial_state, controls);
  double current_violation = problem_.MaxViolation(current_trajectory);

  // penalty_reference_violation: 最近一次“更新罚参数”时的违约量基准。
  // previous_outer_violation: 上一轮外迭代违约量。
  double penalty_reference_violation = current_violation;
  double previous_outer_violation = current_violation;

  // best_trajectory/best_violation 用于可选“返回历史最可行轨迹”。
  double best_violation = current_violation;
  double final_violation = current_violation;
  Trajectory best_trajectory = current_trajectory;
  bool converged = false;

  for (int outer_iter = 0; outer_iter < options_.max_outer_iterations; ++outer_iter) {
    std::cout << "AL-iLQR Outer Iteration " << (outer_iter + 1) << "/" << options_.max_outer_iterations
              << ", Violation: " << current_violation << std::endl;
    // 1) 内层 iLQR：在当前 lambda/mu 固定下，最小化增广代价。
    ILQRSolver inner_solver(*al_problem.UnconstrainedSubproblem(), options_.inner_options);
    current_trajectory = inner_solver.Solve(initial_state, controls);
    current_trajectory.Print();

    // 把当前最优控制作为下一轮 warm start，改善外层收敛效率。
    controls = ExtractControls(current_trajectory);

    // 2) 统计本轮指标（原始代价、增广代价、可行性、罚参数）。
    const double base_cost = problem_.BaseProblem().TotalCost(current_trajectory);
    const double augmented_cost = al_problem.UnconstrainedSubproblem()->TotalCost(current_trajectory);
    const double max_violation = problem_.MaxViolation(current_trajectory);
    const double max_penalty = al_problem.MaxPenalty();
    final_violation = max_violation;

    if (max_violation < best_violation) {
      best_violation = max_violation;
      best_trajectory = current_trajectory;
    }

    outer_log_.push_back(ALILQROuterIterationLog{
        outer_iter + 1,
        static_cast<int>(inner_solver.AlphaHistory().size()),
        base_cost,
        augmented_cost,
        max_violation,
        best_violation,
        max_penalty,
        false,
    });
    std::cout << "  Base Cost: " << base_cost << ", Augmented Cost: " << augmented_cost
              << ", Max Violation: " << max_violation << ", Max Penalty: " << max_penalty
              << ", options_.constraint_tolerance: " << options_.constraint_tolerance
              << std::endl;
    // 3) 约束满足则外层提前收敛。
    if (max_violation <= options_.constraint_tolerance) {
      converged = true;

      break;
    }

    // 4) 不满足则先更新乘子，再按进展情况决定是否放大罚参数。
    al_problem.UpdateDuals(current_trajectory);

    // stagnation_ratio 用于与上一外迭代比较，限制“几乎无进展”的情况。
    const double stagnation_ratio = std::max(options_.penalty_update_ratio, 0.95);
    const bool insufficient_progress_since_penalty =
        max_violation > penalty_reference_violation * options_.penalty_update_ratio;
    const bool insufficient_progress_since_last_outer =
        outer_iter > 0 && max_violation > previous_outer_violation * stagnation_ratio;

    if (insufficient_progress_since_penalty || insufficient_progress_since_last_outer) {
      al_problem.UpdatePenalties();
      outer_log_.back().penalty_updated = true;
      penalty_reference_violation = max_violation;

      // 罚参数超过上限则停止，避免数值过硬导致内层难以优化。
      if (al_problem.MaxPenalty() > options_.max_penalty) {
        break;
      }
    }

    previous_outer_violation = max_violation;
  }

  // 可选返回历史最佳可行轨迹，避免“最后一轮回退”导致结果变差。
  const bool use_best_trajectory = options_.return_best_trajectory && best_violation < final_violation;
  const Trajectory& returned_trajectory = use_best_trajectory ? best_trajectory : current_trajectory;
  const double returned_violation = use_best_trajectory ? best_violation : final_violation;

  return ALILQRResult{returned_trajectory, converged, final_violation, returned_violation, outer_log_};
}

}  // namespace my_al_ilqr
