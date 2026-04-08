#include "al/augmented_lagrangian_cost.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_al_ilqr {

namespace {

// 终端约束评估时使用空控制输入，保持约束接口统一。
Vector EmptyControl() {
  return Vector::Zero(0);
}

// 为单条约束创建 AL 参数初始化：
// - lambda 全零；
// - mu 全为 initial_penalty。
AugmentedLagrangianConstraintData MakeConstraintData(const ConstraintPtr& constraint,
                                                     double initial_penalty,
                                                     double penalty_scaling) {
  if (!constraint) {
    throw std::invalid_argument("Augmented Lagrangian constraint cannot be null.");
  }
  if (initial_penalty <= 0.0 || penalty_scaling <= 1.0) {
    throw std::invalid_argument("Penalty parameters must satisfy mu > 0 and scaling > 1.");
  }
  return AugmentedLagrangianConstraintData{
      constraint,
      Vector::Zero(constraint->OutputDim()),
      Vector::Constant(constraint->OutputDim(), initial_penalty),
      penalty_scaling};
}

// 从约束值向量提取“最大违约量”：
// - 等式约束: max |g_i|
// - 不等式约束(默认 g<=0): max(0, max g_i)
double MaxViolationForValues(ConstraintType type, const Vector& values) {
  if (type == ConstraintType::kEquality) {
    return values.cwiseAbs().maxCoeff();
  }
  double max_violation = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    max_violation = std::max(max_violation, values(i));
  }
  return std::max(0.0, max_violation);
}

}  // namespace

// 构造单个 knot 的增广代价对象。
AugmentedLagrangianKnotCost::AugmentedLagrangianKnotCost(
    std::shared_ptr<CostFunction> base_cost,
    std::vector<AugmentedLagrangianConstraintData> constraints)
    : base_cost_(std::move(base_cost)), constraints_(std::move(constraints)) {
  if (!base_cost_) {
    throw std::invalid_argument("AugmentedLagrangianKnotCost requires a valid base cost.");
  }
}

// 计算单条约束的 AL 增广项。
// 公式：
// - 等式约束 h(x,u)=0:
//     lambda^T h + 0.5 * mu * ||h||^2（逐分量累加）
// - 不等式约束 g(x,u)<=0（采用投影形式）：
//     ((max(0, lambda + mu g))^2 - lambda^2) / (2 mu)
//
// 这里按输出分量逐个累加，支持向量约束。
double AugmentedLagrangianKnotCost::ConstraintAugmentedTerm(
    const AugmentedLagrangianConstraintData& data,
    const Vector& state,
    const Vector& control) const {
  const Vector values = data.constraint->Evaluate(state, control);
  double total = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    if (data.constraint->Type() == ConstraintType::kEquality) {
      total += data.lambda(i) * values(i) + 0.5 * data.mu(i) * values(i) * values(i);
    } else {
      const double projected = std::max(0.0, data.lambda(i) + data.mu(i) * values(i));
      total += (projected * projected - data.lambda(i) * data.lambda(i)) / (2.0 * data.mu(i));
    }
  }
  return total;
}

// 终端增广项调用同一公式，只是控制输入固定为空向量。
double AugmentedLagrangianKnotCost::TerminalAugmentedTerm(
    const AugmentedLagrangianConstraintData& data,
    const Vector& state) const {
  return ConstraintAugmentedTerm(data, state, EmptyControl());
}

// 阶段代价 = 原始阶段代价 + 所有约束的增广项。
double AugmentedLagrangianKnotCost::StageCost(const Vector& state, const Vector& control) const {
  double cost = base_cost_->StageCost(state, control);
  for (const auto& data : constraints_) {
    cost += ConstraintAugmentedTerm(data, state, control);
  }
  return cost;
}

// 终端代价 = 原始终端代价 + 所有终端约束的增广项。
double AugmentedLagrangianKnotCost::TerminalCost(const Vector& state) const {
  double cost = base_cost_->TerminalCost(state);
  for (const auto& data : constraints_) {
    cost += TerminalAugmentedTerm(data, state);
  }
  return cost;
}

// 按当前约束值更新乘子：
// - 等式: lambda <- lambda + mu * h
// - 不等式: lambda <- max(0, lambda + mu * g)
void AugmentedLagrangianKnotCost::UpdateDuals(const Vector& state, const Vector& control) {
  for (auto& data : constraints_) {
    const Vector values = data.constraint->Evaluate(state, control);
    if (data.constraint->Type() == ConstraintType::kEquality) {
      data.lambda += data.mu.cwiseProduct(values);
    } else {
      data.lambda = (data.lambda + data.mu.cwiseProduct(values)).cwiseMax(0.0);
    }
  }
}

// 放大罚参数，增加后续对违约的惩罚力度。
void AugmentedLagrangianKnotCost::UpdatePenalties() {
  for (auto& data : constraints_) {
    data.mu *= data.penalty_scaling;
  }
}

// 返回该 knot 上所有约束的最大违约量。
double AugmentedLagrangianKnotCost::MaxViolation(const Vector& state, const Vector& control) const {
  double max_violation = 0.0;
  for (const auto& data : constraints_) {
    const Vector values = data.constraint->Evaluate(state, control);
    max_violation = std::max(max_violation, MaxViolationForValues(data.constraint->Type(), values));
  }
  return max_violation;
}

// 返回该 knot 所有约束分量中的最大 mu。
double AugmentedLagrangianKnotCost::MaxPenalty() const {
  double max_penalty = 0.0;
  for (const auto& data : constraints_) {
    if (data.mu.size() > 0) {
      max_penalty = std::max(max_penalty, data.mu.maxCoeff());
    }
  }
  return max_penalty;
}

// 基于受约束问题构建增广无约束子问题。
// 关键数据流：
//   constrained_problem
//      -> 为每个时刻打包 stage constraints -> stage AugmentedLagrangianKnotCost
//      -> 打包 terminal constraints -> terminal AugmentedLagrangianKnotCost
//      -> 组合成新的 OptimalControlProblem(subproblem_)
AugmentedLagrangianProblem::AugmentedLagrangianProblem(
    const ConstrainedOptimalControlProblem& constrained_problem,
    double initial_penalty,
    double penalty_scaling) {
  auto base_problem = constrained_problem.SharedBaseProblem();

  std::vector<std::shared_ptr<CostFunction>> stage_costs;
  stage_costs.reserve(base_problem->Horizon());
  stage_costs_.reserve(base_problem->Horizon());
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    std::vector<AugmentedLagrangianConstraintData> constraints;
    for (const auto& constraint : constrained_problem.StageConstraints(k)) {
      constraints.push_back(MakeConstraintData(constraint, initial_penalty, penalty_scaling));
    }
    auto knot_cost =
        std::make_shared<AugmentedLagrangianKnotCost>(base_problem->SharedStageCostFunction(k),
                                                      constraints);
    stage_costs_.push_back(knot_cost);
    stage_costs.push_back(knot_cost);
  }

  std::vector<AugmentedLagrangianConstraintData> terminal_constraints;
  for (const auto& constraint : constrained_problem.TerminalConstraints()) {
    terminal_constraints.push_back(MakeConstraintData(constraint, initial_penalty, penalty_scaling));
  }
  terminal_cost_ =
      std::make_shared<AugmentedLagrangianKnotCost>(base_problem->SharedTerminalCostFunction(),
                                                    terminal_constraints);

  subproblem_ = std::make_shared<OptimalControlProblem>(base_problem->SharedDynamics(),
                                                        std::move(stage_costs),
                                                        terminal_cost_,
                                                        base_problem->TimeStep());
}

// 用整条轨迹更新所有时刻（含终端）的乘子。
void AugmentedLagrangianProblem::UpdateDuals(const Trajectory& trajectory) {
  for (int k = 0; k < static_cast<int>(stage_costs_.size()); ++k) {
    stage_costs_[k]->UpdateDuals(trajectory.State(k), trajectory.Control(k));
  }
  terminal_cost_->UpdateDuals(trajectory.State(trajectory.Horizon()), EmptyControl());
}

// 放大所有时刻（含终端）的罚参数。
void AugmentedLagrangianProblem::UpdatePenalties() {
  for (auto& cost : stage_costs_) {
    cost->UpdatePenalties();
  }
  terminal_cost_->UpdatePenalties();
}

// 统计整条轨迹上的最大违约量。
double AugmentedLagrangianProblem::MaxViolation(const Trajectory& trajectory) const {
  double max_violation = 0.0;
  for (int k = 0; k < static_cast<int>(stage_costs_.size()); ++k) {
    max_violation = std::max(max_violation,
                             stage_costs_[k]->MaxViolation(trajectory.State(k), trajectory.Control(k)));
  }
  max_violation = std::max(max_violation,
                           terminal_cost_->MaxViolation(trajectory.State(trajectory.Horizon()), EmptyControl()));
  return max_violation;
}

// 统计全问题最大罚参数（用于外层停止或安全阈值判断）。
double AugmentedLagrangianProblem::MaxPenalty() const {
  double max_penalty = terminal_cost_->MaxPenalty();
  for (const auto& cost : stage_costs_) {
    max_penalty = std::max(max_penalty, cost->MaxPenalty());
  }
  return max_penalty;
}

}  // namespace my_al_ilqr
