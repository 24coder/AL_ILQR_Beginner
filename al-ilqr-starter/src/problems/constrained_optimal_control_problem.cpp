#include "problems/constrained_optimal_control_problem.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_al_ilqr {

// 构造受约束问题：保存底层问题指针，并按时域长度初始化阶段约束容器。
// stage_constraints_[k] 对应第 k 个离散时刻的约束列表。
ConstrainedOptimalControlProblem::ConstrainedOptimalControlProblem(
    std::shared_ptr<OptimalControlProblem> problem)
    : problem_(std::move(problem)),
      stage_constraints_(problem_ ? problem_->Horizon() : 0) {
  if (!problem_) {
    throw std::invalid_argument("ConstrainedOptimalControlProblem requires a valid base problem.");
  }
}

// 在指定时刻添加阶段约束。
// 维度要求：约束的 StateDim/ControlDim 必须与底层问题一致。
void ConstrainedOptimalControlProblem::AddStageConstraint(int k, ConstraintPtr constraint) {
  if (k < 0 || k >= Horizon()) {
    throw std::out_of_range("Stage constraint index is out of range.");
  }
  if (!constraint) {
    throw std::invalid_argument("Stage constraint pointer cannot be null.");
  }
  if (constraint->StateDim() != problem_->StateDim() || constraint->ControlDim() != problem_->ControlDim()) {
    throw std::invalid_argument("Stage constraint dimensions do not match the base problem.");
  }
  stage_constraints_[k].push_back(std::move(constraint));
}

// 添加终端约束。
// 终端约束只作用在状态 x_N，因此这里只检查 StateDim。
void ConstrainedOptimalControlProblem::AddTerminalConstraint(ConstraintPtr constraint) {
  if (!constraint) {
    throw std::invalid_argument("Terminal constraint pointer cannot be null.");
  }
  if (constraint->StateDim() != problem_->StateDim()) {
    throw std::invalid_argument("Terminal constraint state dimensions do not match the base problem.");
  }
  terminal_constraints_.push_back(std::move(constraint));
}

// 评估轨迹上的所有约束，输出一组 ConstraintEvaluation 记录。
// 数据流：
// 1) 对每个 k 的每个阶段约束评估 values = g_k(x_k, u_k)
// 2) 对每个终端约束评估 values = g_N(x_N)
// 3) 通过 MaxViolation 计算单约束在该时刻的最大违约量
std::vector<ConstraintEvaluation> ConstrainedOptimalControlProblem::EvaluateTrajectory(
    const Trajectory& trajectory) const {
  if (trajectory.Horizon() != Horizon() || trajectory.StateDim() != problem_->StateDim() ||
      trajectory.ControlDim() != problem_->ControlDim()) {
    throw std::invalid_argument("Trajectory dimensions do not match the constrained problem.");
  }

  std::vector<ConstraintEvaluation> evaluations;
  for (int k = 0; k < Horizon(); ++k) {
    for (const auto& constraint : stage_constraints_[k]) {
      const Vector values = constraint->Evaluate(trajectory.State(k), trajectory.Control(k));
      evaluations.push_back(ConstraintEvaluation{
          constraint->Name(), constraint->Type(), k, values, my_al_ilqr::MaxViolation(*constraint, values)});
    }
  }

  // 终端约束统一以“空控制向量”调用接口，保持 Evaluate(state, control) 的函数签名一致。
  const Vector empty_control = Vector::Zero(0);
  for (const auto& constraint : terminal_constraints_) {
    const Vector values = constraint->Evaluate(trajectory.State(Horizon()), empty_control);
    evaluations.push_back(ConstraintEvaluation{
        constraint->Name(), constraint->Type(), Horizon(), values, my_al_ilqr::MaxViolation(*constraint, values)});
  }

  return evaluations;
}

// 汇总整条轨迹的最大违约量。
// 即对 EvaluateTrajectory 返回的每条记录取 max(evaluation.max_violation)。
double ConstrainedOptimalControlProblem::MaxViolation(const Trajectory& trajectory) const {
  const auto evaluations = EvaluateTrajectory(trajectory);
  double max_violation = 0.0;
  for (const auto& evaluation : evaluations) {
    max_violation = std::max(max_violation, evaluation.max_violation);
  }
  return max_violation;
}

}  // namespace my_al_ilqr
