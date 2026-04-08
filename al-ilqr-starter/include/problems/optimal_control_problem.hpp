#pragma once

#include <memory>
#include <vector>

#include "core/trajectory.hpp"
#include "cost/cost_function.hpp"
#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

// 最优控制问题定义：
// 在离散时域 k=0...N 上，给定动力学与代价，评估或 rollout 轨迹。
class OptimalControlProblem {
 public:
  // 便捷构造：阶段代价与终端代价共用同一个 CostFunction 对象。
  // 内部会复制 horizon 份 shared_ptr 作为 stage_costs_。
  OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                        std::shared_ptr<CostFunction> cost,
                        int horizon,
                        double dt);

  // 通用构造：每个阶段可使用不同代价函数，并单独指定终端代价。
  OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                        std::vector<std::shared_ptr<CostFunction>> stage_costs,
                        std::shared_ptr<CostFunction> terminal_cost,
                        double dt);

  // 基本维度信息由动力学模型定义。
  int StateDim() const;
  int ControlDim() const;

  // 时域长度 N 与时间步长 dt。
  int Horizon() const { return horizon_; }
  double TimeStep() const { return dt_; }

  // 动力学访问接口。
  const DynamicsModel& Dynamics() const { return *dynamics_; }
  std::shared_ptr<DynamicsModel> SharedDynamics() const { return dynamics_; }

  // 代价访问接口。
  const CostFunction& StageCostFunction(int k) const;
  std::shared_ptr<CostFunction> SharedStageCostFunction(int k) const { return stage_costs_.at(k); }
  const CostFunction& TerminalCostFunction() const { return *terminal_cost_; }
  std::shared_ptr<CostFunction> SharedTerminalCostFunction() const { return terminal_cost_; }

  // 给定初始状态与控制序列，按动力学逐步推进，生成完整轨迹。
  Trajectory Rollout(const Vector& initial_state,
                     const std::vector<Vector>& control_sequence) const;

  // 计算轨迹总成本：
  // sum_{k=0}^{N-1} l_k(x_k, u_k) + l_f(x_N)
  double TotalCost(const Trajectory& trajectory) const;

 private:
  // 动力学模型。
  std::shared_ptr<DynamicsModel> dynamics_;

  // 阶段代价列表，长度为 N。
  std::vector<std::shared_ptr<CostFunction>> stage_costs_;

  // 终端代价。
  std::shared_ptr<CostFunction> terminal_cost_;

  // 时域长度。
  int horizon_;

  // 离散时间步长。
  double dt_;
};

}  // namespace my_al_ilqr
