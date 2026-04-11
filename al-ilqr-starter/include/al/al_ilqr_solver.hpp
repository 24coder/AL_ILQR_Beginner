#pragma once

#include <optional>
#include <vector>

#include "al/augmented_lagrangian_cost.hpp"
#include "core/trajectory.hpp"
#include "ilqr/ilqr_solver.hpp"

namespace my_al_ilqr {

// AL-iLQR 外层配置参数：
// - inner_options: 内层 iLQR 的求解超参数
// - 其余参数控制增广拉格朗日外循环（乘子与罚参数更新）
struct ALILQROptions {
  ILQROptions inner_options;
  int max_outer_iterations = 10;
  double constraint_tolerance = 1e-3;
  double initial_penalty = 10.0;
  double penalty_scaling = 5.0;
  double penalty_update_ratio = 0.25;
  double max_penalty = 1e8;
  bool return_best_trajectory = true;
};

// 外层每轮迭代日志，便于分析”可行性收敛”和”罚参数演化”。
struct ALILQROuterIterationLog {
  int outer_iteration = 0;
  int inner_iterations = 0;
  double base_cost = 0.0;
  double augmented_cost = 0.0;
  double max_violation = 0.0;
  double best_violation_so_far = 0.0;
  double max_penalty = 0.0;
  bool penalty_updated = false;
  // 内层 iLQR 每个被接受的迭代的增广代价（包含初始 rollout 代价）。
  // 可用于绘制”内层优化收敛曲线”，验证每轮 AL 子问题的 iLQR 收敛情况。
  std::vector<double> inner_cost_history;
  // 内层 iLQR 每个被接受的迭代所使用的线搜索步长 alpha。
  std::vector<double> inner_alpha_history;
  // 该外层迭代内层 iLQR 求解完毕后的完整轨迹快照。
  // 可用于绘制”轨迹随 AL 迭代演化”的动态过程。
  std::optional<Trajectory> trajectory;
};

// AL-iLQR 求解结果。
struct ALILQRResult {
  Trajectory trajectory;
  bool converged = false;

  // 最后一轮外迭代轨迹的违约量。
  double final_violation = 0.0;

  // 返回轨迹对应的违约量。
  // 当 return_best_trajectory=true 时，它可能小于 final_violation。
  double best_violation = 0.0;

  // 外层日志序列。
  std::vector<ALILQROuterIterationLog> outer_logs;
};

// AL-iLQR 主求解器。
// 框架：
// 1) 由约束问题构建增广子问题；
// 2) 每轮外迭代调用一次内层 iLQR；
// 3) 根据约束收敛情况更新 lambda / mu。
class ALILQRSolver {
 public:
  explicit ALILQRSolver(const ConstrainedOptimalControlProblem& problem, ALILQROptions options = {});

  const std::vector<ALILQROuterIterationLog>& OuterLog() const { return outer_log_; }

  ALILQRResult Solve(const Vector& initial_state, const std::vector<Vector>& initial_controls);

 private:
  const ConstrainedOptimalControlProblem& problem_;
  ALILQROptions options_;
  std::vector<ALILQROuterIterationLog> outer_log_;
};

}  // namespace my_al_ilqr
