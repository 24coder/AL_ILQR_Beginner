#include "problems/optimal_control_problem.hpp"

#include <stdexcept>

namespace my_al_ilqr {

// 便捷构造：使用同一个 cost 对象作为所有阶段代价与终端代价。
// 数据流：
//   输入 (dynamics, cost, horizon, dt)
//      -> stage_costs_ = [cost, cost, ..., cost] (长度 horizon)
//      -> terminal_cost_ = cost
OptimalControlProblem::OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                                             std::shared_ptr<CostFunction> cost,
                                             int horizon,
                                             double dt)
    : dynamics_(std::move(dynamics)),
      stage_costs_(horizon, cost),
      terminal_cost_(cost),
      horizon_(horizon),
      dt_(dt) {
  if (!dynamics_ || !terminal_cost_) {
    throw std::invalid_argument("Problem requires valid dynamics and cost objects.");
  }
  for (const auto& stage_cost : stage_costs_) {
    if (!stage_cost) {
      throw std::invalid_argument("All stage costs must be valid.");
    }
  }
  if (stage_costs_.empty()) {
    throw std::invalid_argument("Problem horizon and cost sequence must be positive.");
  }

  // 保证动力学与代价在 (nx, nu) 维度上一致。
  if (dynamics_->StateDim() != terminal_cost_->StateDim() ||
      dynamics_->ControlDim() != terminal_cost_->ControlDim()) {
    throw std::invalid_argument("Dynamics and terminal cost dimensions are inconsistent.");
  }
  for (const auto& stage_cost : stage_costs_) {
    if (dynamics_->StateDim() != stage_cost->StateDim() ||
        dynamics_->ControlDim() != stage_cost->ControlDim()) {
      throw std::invalid_argument("Dynamics and stage cost dimensions are inconsistent.");
    }
  }
  if (horizon_ <= 0 || dt_ <= 0.0) {
    throw std::invalid_argument("Problem horizon and time step must be positive.");
  }
}

// 通用构造：允许每个阶段的代价函数不同。
OptimalControlProblem::OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                                             std::vector<std::shared_ptr<CostFunction>> stage_costs,
                                             std::shared_ptr<CostFunction> terminal_cost,
                                             double dt)
    : dynamics_(std::move(dynamics)),
      stage_costs_(std::move(stage_costs)),
      terminal_cost_(std::move(terminal_cost)),
      horizon_(static_cast<int>(stage_costs_.size())),
      dt_(dt) {
  if (!dynamics_ || !terminal_cost_) {
    throw std::invalid_argument("Problem requires valid dynamics and cost objects.");
  }
  if (horizon_ <= 0 || dt_ <= 0.0) {
    throw std::invalid_argument("Problem horizon and time step must be positive.");
  }
  if (stage_costs_.empty()) {
    throw std::invalid_argument("Stage cost sequence must be non-empty.");
  }

  // 维度一致性检查：动力学输出/输入维度必须匹配所有代价函数。
  if (dynamics_->StateDim() != terminal_cost_->StateDim() ||
      dynamics_->ControlDim() != terminal_cost_->ControlDim()) {
    throw std::invalid_argument("Dynamics and terminal cost dimensions are inconsistent.");
  }
  for (const auto& stage_cost : stage_costs_) {
    if (!stage_cost) {
      throw std::invalid_argument("All stage costs must be valid.");
    }
    if (dynamics_->StateDim() != stage_cost->StateDim() ||
        dynamics_->ControlDim() != stage_cost->ControlDim()) {
      throw std::invalid_argument("Dynamics and stage cost dimensions are inconsistent.");
    }
  }
}

int OptimalControlProblem::StateDim() const {
  return dynamics_->StateDim();
}

int OptimalControlProblem::ControlDim() const {
  return dynamics_->ControlDim();
}

// 返回第 k 个阶段代价函数（带边界检查）。
const CostFunction& OptimalControlProblem::StageCostFunction(int k) const {
  return *stage_costs_.at(k);
}

// 根据给定控制序列进行前向 rollout：
//   x_{k+1} = f(x_k, u_k, dt)
// 并把 (x_k, u_k) 全部写入 Trajectory。
Trajectory OptimalControlProblem::Rollout(const Vector& initial_state,
                                          const std::vector<Vector>& control_sequence) const {
  if (initial_state.size() != StateDim()) {
    throw std::invalid_argument("Initial state has invalid dimensions.");
  }
  if (static_cast<int>(control_sequence.size()) != horizon_) {
    throw std::invalid_argument("Control sequence length must match the problem horizon.");
  }

  Trajectory trajectory(StateDim(), ControlDim(), horizon_);
  trajectory.SetTimeStep(dt_);
  trajectory.State(0) = initial_state;

  for (int k = 0; k < horizon_; ++k) {
    if (control_sequence[k].size() != ControlDim()) {
      throw std::invalid_argument("Control input has invalid dimensions.");
    }
    trajectory.Control(k) = control_sequence[k];
    trajectory.State(k + 1) = dynamics_->NextState(trajectory.State(k), trajectory.Control(k), dt_);
  }
  return trajectory;
}

// 计算整条轨迹总代价。
// 这里严格按离散时域定义累加：
// - 阶段代价使用 k=0...N-1 的 (x_k, u_k)
// - 终端代价使用 x_N
double OptimalControlProblem::TotalCost(const Trajectory& trajectory) const {
  if (trajectory.StateDim() != StateDim() || trajectory.ControlDim() != ControlDim() ||
      trajectory.Horizon() != horizon_) {
    throw std::invalid_argument("Trajectory dimensions do not match the problem definition.");
  }

  double total_cost = 0.0;
  for (int k = 0; k < horizon_; ++k) {
    total_cost += stage_costs_.at(k)->StageCost(trajectory.State(k), trajectory.Control(k));
  }
  total_cost += terminal_cost_->TerminalCost(trajectory.State(horizon_));
  return total_cost;
}

}  // namespace my_al_ilqr
