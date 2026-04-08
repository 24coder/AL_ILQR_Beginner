#pragma once

#include <memory>
#include <vector>

#include "constraints/constraint_function.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

// 在无约束最优控制问题之上叠加约束集合：
// - 阶段约束 g_k(x_k, u_k)
// - 终端约束 g_N(x_N)
//
// 本类负责“约束组织与评估”，不直接执行优化。
class ConstrainedOptimalControlProblem {
 public:
  explicit ConstrainedOptimalControlProblem(std::shared_ptr<OptimalControlProblem> problem);

  // 访问底层无约束问题。
  const OptimalControlProblem& BaseProblem() const { return *problem_; }
  std::shared_ptr<OptimalControlProblem> SharedBaseProblem() const { return problem_; }

  // 时域长度与底层问题保持一致。
  int Horizon() const { return problem_->Horizon(); }

  // 访问第 k 个时刻绑定的阶段约束列表。
  const std::vector<ConstraintPtr>& StageConstraints(int k) const { return stage_constraints_.at(k); }

  // 访问终端约束列表。
  const std::vector<ConstraintPtr>& TerminalConstraints() const { return terminal_constraints_; }

  // 向第 k 个时刻添加阶段约束。
  void AddStageConstraint(int k, ConstraintPtr constraint);

  // 添加终端约束。
  void AddTerminalConstraint(ConstraintPtr constraint);

  // 评估整条轨迹上所有约束，并返回逐条评估结果。
  std::vector<ConstraintEvaluation> EvaluateTrajectory(const Trajectory& trajectory) const;

  // 返回轨迹整体最大违约量（所有约束、所有时刻的 max）。
  double MaxViolation(const Trajectory& trajectory) const;

 private:
  // 底层无约束问题。
  std::shared_ptr<OptimalControlProblem> problem_;

  // 每个时刻对应一组阶段约束，长度为 Horizon()。
  std::vector<std::vector<ConstraintPtr>> stage_constraints_;

  // 终端约束集合（作用在 x_N）。
  std::vector<ConstraintPtr> terminal_constraints_;
};

}  // namespace my_al_ilqr
