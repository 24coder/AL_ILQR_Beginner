#pragma once

#include <memory>
#include <vector>

#include "constraints/constraint_function.hpp"
#include "cost/cost_function.hpp"
#include "problems/constrained_optimal_control_problem.hpp"

namespace my_al_ilqr {

// 增广拉格朗日中“单条约束”的可变参数与元数据。
// 对每个约束输出分量 i，维护：
// - lambda_i: 拉格朗日乘子
// - mu_i: 罚参数
// - penalty_scaling: 每次放大 mu 时的倍数（通常 > 1）
struct AugmentedLagrangianConstraintData {
  ConstraintPtr constraint;
  Vector lambda;
  Vector mu;
  double penalty_scaling = 10.0;
};

// 单个 knot（阶段或终端）上的“增广代价”。
// 它包装 base_cost，并把若干约束的增广项叠加进去，形成可直接给 iLQR 求解的无约束代价。
class AugmentedLagrangianKnotCost final : public CostFunction {
 public:
  AugmentedLagrangianKnotCost(std::shared_ptr<CostFunction> base_cost,
                              std::vector<AugmentedLagrangianConstraintData> constraints);

  // 维度与 base_cost 保持一致。
  int StateDim() const override { return base_cost_->StateDim(); }
  int ControlDim() const override { return base_cost_->ControlDim(); }

  // 阶段代价：base_cost.StageCost + 所有阶段约束增广项。
  double StageCost(const Vector& state, const Vector& control) const override;

  // 终端代价：base_cost.TerminalCost + 所有终端约束增广项。
  double TerminalCost(const Vector& state) const override;

  // 按当前轨迹点更新乘子 lambda。
  void UpdateDuals(const Vector& state, const Vector& control);

  // 放大罚参数 mu。
  void UpdatePenalties();

  // 返回该 knot 上所有约束的最大违约量。
  double MaxViolation(const Vector& state, const Vector& control) const;

  // 返回该 knot 上所有约束分量中的最大罚参数。
  double MaxPenalty() const;

  const std::vector<AugmentedLagrangianConstraintData>& Constraints() const { return constraints_; }
  std::vector<AugmentedLagrangianConstraintData>& Constraints() { return constraints_; }

 private:
  // 计算单条约束在阶段点 (x_k, u_k) 的增广项。
  double ConstraintAugmentedTerm(const AugmentedLagrangianConstraintData& data,
                                 const Vector& state,
                                 const Vector& control) const;

  // 计算单条约束在终端点 x_N 的增广项。
  double TerminalAugmentedTerm(const AugmentedLagrangianConstraintData& data,
                               const Vector& state) const;

  // 原始代价。
  std::shared_ptr<CostFunction> base_cost_;

  // 该 knot 绑定的约束列表及其 AL 参数。
  std::vector<AugmentedLagrangianConstraintData> constraints_;
};

// 把受约束 OCP 转成“可反复更新”的增广拉格朗日无约束子问题。
// 外层 AL 循环负责更新 lambda/mu；内层 iLQR 每轮求解当前子问题。
class AugmentedLagrangianProblem {
 public:
  AugmentedLagrangianProblem(const ConstrainedOptimalControlProblem& constrained_problem,
                             double initial_penalty,
                             double penalty_scaling);

  // 当前可直接交给 iLQR 的无约束子问题。
  std::shared_ptr<OptimalControlProblem> UnconstrainedSubproblem() const { return subproblem_; }

  // 访问某个阶段/终端 knot 的增广代价对象（便于调试或外部观测）。
  std::shared_ptr<AugmentedLagrangianKnotCost> StageCost(int k) const { return stage_costs_.at(k); }
  std::shared_ptr<AugmentedLagrangianKnotCost> TerminalCost() const { return terminal_cost_; }

  // 用一条轨迹更新所有 knot 上的乘子。
  void UpdateDuals(const Trajectory& trajectory);

  // 放大全部 knot 的罚参数。
  void UpdatePenalties();

  // 计算整条轨迹在所有约束上的最大违约量。
  double MaxViolation(const Trajectory& trajectory) const;

  // 返回全问题当前最大罚参数。
  double MaxPenalty() const;

 private:
  // 当前无约束子问题（动力学不变，代价为增广代价）。
  std::shared_ptr<OptimalControlProblem> subproblem_;

  // 各阶段的增广代价对象，长度为 N。
  std::vector<std::shared_ptr<AugmentedLagrangianKnotCost>> stage_costs_;

  // 终端增广代价对象。
  std::shared_ptr<AugmentedLagrangianKnotCost> terminal_cost_;
};

}  // namespace my_al_ilqr
