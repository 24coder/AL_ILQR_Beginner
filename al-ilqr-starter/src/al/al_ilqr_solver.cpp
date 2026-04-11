#include "al/al_ilqr_solver.hpp"

#include <stdexcept>
#include <iostream>

namespace my_al_ilqr {

namespace {

// 从 Trajectory 中提取控制序列 u_0, u_1, ..., u_{N-1}。
//
// 设计动机：
// 在 AL-iLQR 中，外层每一轮都会调用一次内层 iLQR 求解。
// 内层求解完成后，我们得到一条新的轨迹：
//   trajectory = {x_0, u_0, x_1, u_1, ..., x_N}
// 下一轮外层迭代通常会继续在“上一轮已经优化过的控制序列”附近搜索，
// 这就是所谓的 warm start（热启动）。
//
// 因此，这个辅助函数的作用就是：
// - 从完整轨迹对象中把每个时刻的控制量取出来；
// - 组装成 std::vector<Vector>；
// - 作为下一轮内层 iLQR 的初始控制猜测。
//
// 为什么不直接复用旧 controls？
// 因为 inner_solver.Solve(...) 之后，轨迹里的控制已经被更新成更优结果，
// 用更新后的控制继续下一轮，可以显著提升外层 AL 的收敛效率和稳定性。
std::vector<Vector> ExtractControls(const Trajectory& trajectory) {
  // 轨迹长度为 N，则存在 N 个控制量 u_0...u_{N-1}，
  // 控制维度由 trajectory.ControlDim() 给出。
  std::vector<Vector> controls(trajectory.Horizon(), Vector::Zero(trajectory.ControlDim()));

  // 逐时刻读取轨迹中的控制输入。
  for (int k = 0; k < trajectory.Horizon(); ++k) {
    controls[k] = trajectory.Control(k);
  }

  return controls;
}

}  // namespace

// 构造 AL-iLQR 求解器。
//
// 该类负责求解“带约束最优控制问题”：
// - 外层：Augmented Lagrangian (AL) 处理约束
// - 内层：iLQR 处理在固定乘子/罚参数下的无约束增广子问题
//
// 这里在构造阶段做参数合法性检查，属于 fail-fast 设计：
// 一旦发现配置明显错误，立刻抛异常，而不是等到迭代过程中产生更隐蔽的数值问题。
ALILQRSolver::ALILQRSolver(const ConstrainedOptimalControlProblem& problem, ALILQROptions options)
    : problem_(problem), options_(std::move(options)) {
  // 外层迭代次数必须为正。
  // 如果 <= 0，则算法根本没有机会执行任何 AL 更新。
  if (options_.max_outer_iterations <= 0) {
    throw std::invalid_argument("AL-iLQR max_outer_iterations must be positive.");
  }

  // 约束容差必须为正。
  // 该值决定“多小的违反量可以被视为已满足约束”。
  // 若 <= 0，通常意味着要求绝对零违反，这在数值优化中既不现实也不稳定。
  if (options_.constraint_tolerance <= 0.0) {
    throw std::invalid_argument("AL-iLQR constraint_tolerance must be positive.");
  }

  // 初始罚参数必须为正；罚参数放大倍数必须 > 1。
  // - initial_penalty <= 0 会使增广二次罚项失去意义；
  // - penalty_scaling <= 1 则意味着“更新罚参数”时不会真正增大，
  //   无法体现 AL 在约束迟迟不下降时逐步加强惩罚的机制。
  if (options_.initial_penalty <= 0.0 || options_.penalty_scaling <= 1.0) {
    throw std::invalid_argument("AL-iLQR penalty parameters must satisfy mu > 0 and scaling > 1.");
  }

  // penalty_update_ratio 必须位于 (0, 1) 内。
  // 它表示：如果当前违反量相对于某个参考违反量没有下降到足够比例，
  // 就认为“约束改善不够明显”，需要提高罚参数。
  //
  // 例如 ratio = 0.8 时，意味着若违反没有至少下降到原来的 80% 以下，
  // 就可能触发 penalty update。
  if (options_.penalty_update_ratio <= 0.0 || options_.penalty_update_ratio >= 1.0) {
    throw std::invalid_argument("AL-iLQR penalty_update_ratio must lie in (0, 1).");
  }

  // 罚参数上限必须为正。
  // 该上限用于防止罚参数无限增大，从而导致增广问题病态、
  // Hessian 条件数恶化、内层 iLQR 难以稳定优化。
  if (options_.max_penalty <= 0.0) {
    throw std::invalid_argument("AL-iLQR max_penalty must be positive.");
  }
}

// AL-iLQR 主求解流程。
//
// 输入：
// - initial_state：系统初始状态 x_0
// - initial_controls：初始控制序列 {u_0, ..., u_{N-1}}
//
// 输出：
// - 最终轨迹
// - 是否收敛
// - 最终违反量 / 返回轨迹对应的违反量
// - 每轮外层迭代日志
//
// 算法框架可以概括为：
//
//   初始化 lambda, mu, trajectory
//   for outer_iter = 1 ... max_outer_iterations:
//       1) 固定当前 lambda / mu，构造增广无约束子问题
//       2) 调用 iLQR 最小化该子问题，得到新轨迹
//       3) 评估原始代价、增广代价、约束违反量
//       4) 若违反量足够小，则收敛退出
//       5) 否则更新拉格朗日乘子 lambda
//       6) 如果可行性进展不足，则提高罚参数 mu
//   返回最后轨迹或历史最优可行轨迹
ALILQRResult ALILQRSolver::Solve(const Vector& initial_state,
                                 const std::vector<Vector>& initial_controls) {
  // ---------- 输入维度检查 ----------
  // 初始状态维度必须与基础最优控制问题的状态维度一致。
  if (initial_state.size() != problem_.BaseProblem().StateDim()) {
    throw std::invalid_argument("AL-iLQR received an initial state with invalid dimensions.");
  }

  // 初始控制序列长度必须与优化时域长度一致。
  // 对于 horizon = N，必须提供 N 个控制输入 u_0...u_{N-1}。
  if (static_cast<int>(initial_controls.size()) != problem_.Horizon()) {
    throw std::invalid_argument("AL-iLQR received a control sequence with invalid length.");
  }

  // 清空上一次求解遗留的外层日志，确保本次 Solve 的结果独立。
  outer_log_.clear();

  // ---------- 构造初始增广问题 ----------
  // AugmentedLagrangianProblem 会基于：
  // - 原始带约束问题 problem_
  // - 初始罚参数 options_.initial_penalty
  // - 罚参数放大倍数 options_.penalty_scaling
  // 来维护增广拉格朗日子问题及其乘子/罚参数更新逻辑。
  //
  // 注意：这里将 penalty_scaling 也传进去，说明该对象内部不仅保存当前 penalty，
  // 还知道“如何在后续 UpdatePenalties() 时放大 penalty”。
  AugmentedLagrangianProblem al_problem(problem_, options_.initial_penalty, options_.penalty_scaling);

  // 当前控制序列初始化为用户给定的初始控制。
  // 后续每轮外层迭代结束后，都会被替换为最新轨迹对应的控制序列，用于 warm start。
  std::vector<Vector> controls = initial_controls;

  // 用当前控制序列对系统进行 rollout，得到初始轨迹。
  // 这条轨迹还没有经过当前 AL 子问题优化，只是“用户初值”对应的轨迹。
  Trajectory current_trajectory = problem_.BaseProblem().Rollout(initial_state, controls);

  // 计算初始轨迹的最大约束违反量。
  // 这是一个非常重要的量：
  // - 用于判断是否已经近似可行
  // - 用于决定何时增大罚参数
  // - 用于记录历史最好可行性结果
  double current_violation = problem_.MaxViolation(current_trajectory);

  // penalty_reference_violation：
  // 最近一次“更新罚参数时”的违反量参考值。
  //
  // 作用：
  // 后面会比较当前违反量是否相对于这个参考值“下降足够多”。
  // 如果没有明显下降，则说明当前 penalty 强度不足，需要继续增大。
  double penalty_reference_violation = current_violation;

  // previous_outer_violation：
  // 记录上一轮外层迭代结束时的违反量。
  //
  // 作用：
  // 用于检测连续两轮外层之间是否出现“停滞”（stagnation），
  // 即虽然还没到需要和 penalty reference 比较的程度，但最近一轮几乎没改善。
  double previous_outer_violation = current_violation;

  // best_trajectory / best_violation：
  // 用于追踪整个求解过程中“历史上违反最小”的轨迹。
  //
  // 为什么需要这个机制？
  // 因为外层最后一轮未必是最可行的一轮：
  // - 可能最后一轮由于 penalty 更新、数值波动等原因，违反量反而略有回升；
  // - 如果直接返回最后轨迹，结果可能比前面某一轮更差。
  //
  // 因此可选地返回 best_trajectory，会更稳健。
  double best_violation = current_violation;

  // final_violation：
  // 记录“最后一轮实际迭代得到的轨迹”的违反量。
  // 注意它不一定等于返回轨迹的违反量，因为返回的可能是历史最佳轨迹。
  double final_violation = current_violation;

  // 初始化历史最佳轨迹为当前初始 rollout 得到的轨迹。
  Trajectory best_trajectory = current_trajectory;

  // converged 标记是否因满足约束容差而成功收敛。
  bool converged = false;

  // ---------- 外层 AL 迭代 ----------
  for (int outer_iter = 0; outer_iter < options_.max_outer_iterations; ++outer_iter) {
    // 打印本轮外层迭代编号与当前违反量，便于实时观察可行性收敛过程。
    std::cout << "AL-iLQR Outer Iteration " << (outer_iter + 1) << "/" << options_.max_outer_iterations
              << ", Violation: " << current_violation << std::endl;

    // -----------------------------------------------------------------
    // 1) 求解当前增广无约束子问题（内层 iLQR）
    // -----------------------------------------------------------------
    // 在外层 AL 看来，此时当前的拉格朗日乘子 lambda 和罚参数 mu 是固定的，
    // 所以可以得到一个“增广后的无约束最优控制问题”。
    //
    // 内层 iLQR 就是在这个无约束子问题上做局部二次近似 + 迭代优化。
    ILQRSolver inner_solver(*al_problem.UnconstrainedSubproblem(), options_.inner_options);

    // 用当前控制序列 controls 作为 warm start，求解得到新的轨迹。
    current_trajectory = inner_solver.Solve(initial_state, controls);

    // 将当前轨迹中的控制提取出来，作为下一轮外层迭代的初值。
    // 这样外层每轮都不是“从头开始”，而是在上一轮结果基础上继续细化。
    controls = ExtractControls(current_trajectory);

    // -----------------------------------------------------------------
    // 2) 统计并记录本轮的关键指标
    // -----------------------------------------------------------------
    // 原始问题代价：不含 AL 罚项，仅看真正的任务目标代价。
    const double base_cost = problem_.BaseProblem().TotalCost(current_trajectory);

    // 增广问题代价：包含约束对应的乘子项、罚项。
    // 这个值是内层 iLQR 实际在优化的目标。
    const double augmented_cost = al_problem.UnconstrainedSubproblem()->TotalCost(current_trajectory);

    // 当前轨迹的最大约束违反量。
    // AL 收敛最核心的指标之一。
    const double max_violation = problem_.MaxViolation(current_trajectory);

    // 当前所有约束项中最大的罚参数。
    // 用于监控 penalty 是否持续上升以及是否接近上限。
    const double max_penalty = al_problem.MaxPenalty();

    // 记录“最后一轮求解结果”的违反量。
    final_violation = max_violation;

    // 如果当前轨迹比历史最佳轨迹更可行，则更新 best 记录。
    if (max_violation < best_violation) {
      best_violation = max_violation;
      best_trajectory = current_trajectory;
    }

    // 将本轮外层迭代的重要统计信息写入日志。
    // inner_iterations 这里通过 AlphaHistory().size() 获得，
    // 说明实现中把每次成功/尝试的 line-search alpha 历史作为迭代痕迹之一。
    outer_log_.push_back(ALILQROuterIterationLog{
        outer_iter + 1,
        static_cast<int>(inner_solver.AlphaHistory().size()),
        base_cost,
        augmented_cost,
        max_violation,
        best_violation,
        max_penalty,
        false,
        // 保存内层 iLQR 的 cost/alpha 历史，用于"iLQR 内层收敛曲线"可视化。
        inner_solver.CostHistory(),
        inner_solver.AlphaHistory(),
        // 保存本轮 AL 迭代后的轨迹快照，用于"轨迹演化"可视化。
        current_trajectory,
    });

    // 控制台摘要输出，便于直接观察每轮的代价和可行性变化趋势。
    std::cout << "  Base Cost: " << base_cost << ", Augmented Cost: " << augmented_cost
              << ", Max Violation: " << max_violation << ", Max Penalty: " << max_penalty
              << ", options_.constraint_tolerance: " << options_.constraint_tolerance
              << std::endl;

    // -----------------------------------------------------------------
    // 3) 收敛判定：若约束违反已足够小，则外层提前结束
    // -----------------------------------------------------------------
    // 这里的收敛标准是“最大违反量 <= 约束容差”。
    // 这强调的是可行性收敛；通常默认内层 iLQR 已经在给定增广目标下做了局部优化。
    if (max_violation <= options_.constraint_tolerance) {
      converged = true;
      break;
    }

    // -----------------------------------------------------------------
    // 4) 未收敛：更新乘子，再视进展情况决定是否增加罚参数
    // -----------------------------------------------------------------
    // 先更新对偶变量（拉格朗日乘子）。
    // 这是 AL 的核心步骤之一：
    // - 如果某些约束仍被违反，相应乘子会被调整，
    //   从而在下一轮增广目标中更强烈地反映该约束的重要性。
    // - 与纯 penalty method 相比，AL 往往能在不把 penalty 提到极大时，
    //   也有效推动可行性收敛。
    al_problem.UpdateDuals(current_trajectory);

    // stagnation_ratio：
    // 用来和“上一轮外层违反量”比较的停滞阈值。
    //
    // 这里取 max(penalty_update_ratio, 0.95)，意味着：
    // - 如果用户设置的 penalty_update_ratio 比较宽松（例如 0.8），
    //   那么和上一轮比较时仍采用更保守的 0.95；
    // - 即：若相邻两轮之间违反量下降不到 5%，就可视为几乎停滞。
    //
    // 这样做的好处是：
    // - 避免只依赖“相对于上次 penalty 更新时”的远距离比较；
    // - 能更快识别最近几轮已经进入平台期的问题。
    const double stagnation_ratio = std::max(options_.penalty_update_ratio, 0.95);

    // 判断 1：相对于“最近一次 penalty 更新时的参考违反量”，
    // 当前违反量是否下降得不够多。
    //
    // 举例：
    // penalty_reference_violation = 1.0, ratio = 0.8
    // 如果当前 violation 仍然 > 0.8，就说明下降不足 20%，可能需要增大 penalty。
    const bool insufficient_progress_since_penalty =
        max_violation > penalty_reference_violation * options_.penalty_update_ratio;

    // 判断 2：相对于“上一轮外层迭代”的违反量，是否几乎没有进展。
    //
    // 只有 outer_iter > 0 时才有上一轮可比较。
    // 若 max_violation > previous_outer_violation * stagnation_ratio，
    // 则说明最近一轮下降太少，可能已经陷入停滞，需要提高 penalty 强度。
    const bool insufficient_progress_since_last_outer =
        outer_iter > 0 && max_violation > previous_outer_violation * stagnation_ratio;

    // 如果任一条件成立，说明当前罚参数可能不足以继续有效推动可行性改善。
    if (insufficient_progress_since_penalty || insufficient_progress_since_last_outer) {
      // 增大罚参数。
      // 典型效果是让违反约束的代价在下一轮中更“陡峭”，
      // 从而迫使内层 iLQR 更重视可行性而非单纯优化原始代价。
      al_problem.UpdatePenalties();

      // 记录日志：本轮发生了 penalty 更新。
      outer_log_.back().penalty_updated = true;

      // 更新 penalty reference violation。
      // 因为从此刻开始，后续“相对上次 penalty 更新以来进展如何”的比较，
      // 都应该以当前 violation 为新的参考基准。
      penalty_reference_violation = max_violation;

      // 若罚参数超过允许上限，则停止迭代。
      // 原因：
      // - 过大的罚参数会使问题严重病态；
      // - 增广 Hessian 可能变得数值上非常僵硬；
      // - 内层 iLQR 可能越来越难以前进。
      //
      // 这里采取的是“保守停止”策略，而不是无穷无尽地继续放大。
      if (al_problem.MaxPenalty() > options_.max_penalty) {
        break;
      }
    }

    // 更新“上一轮外层违反量”，供下一轮判断停滞使用。
    previous_outer_violation = max_violation;

    // 注意：这里没有显式更新 current_violation。
    // 这是因为当前实现主要在循环顶部打印旧值，而真正用于判断和记录的是 max_violation。
    // 如果想让下一轮开头打印的是最新违反量，可以在这里写：
    //   current_violation = max_violation;
    //
    // 目前这种写法不影响核心求解逻辑，因为后续决策均基于 max_violation / previous_outer_violation
    // / penalty_reference_violation 等最新变量，而不是 current_violation 本身。
  }

  // ---------- 决定最终返回哪条轨迹 ----------
  // 如果启用了 return_best_trajectory，且历史最佳违反量比最后一轮更小，
  // 则返回历史最佳轨迹；否则返回最后一轮轨迹。
  //
  // 这是一个很实用的保护机制：
  // 避免由于最后一步 penalty 更新或局部数值波动，导致“最后结果不如中间某轮”。
  const bool use_best_trajectory = options_.return_best_trajectory && best_violation < final_violation;
  const Trajectory& returned_trajectory = use_best_trajectory ? best_trajectory : current_trajectory;
  const double returned_violation = use_best_trajectory ? best_violation : final_violation;

  // ---------- 打包返回结果 ----------
  // 返回值同时包含：
  // - returned_trajectory：最终决定返回给用户的轨迹
  // - converged：是否按容差标准收敛
  // - final_violation：最后一轮轨迹的违反量
  // - returned_violation：返回轨迹的违反量（可能优于 final_violation）
  // - outer_log_：完整外层日志
  return ALILQRResult{returned_trajectory, converged, final_violation, returned_violation, outer_log_};
  
}

}  // namespace my_al_ilqr
