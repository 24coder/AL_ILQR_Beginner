#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

// 代价函数抽象接口。
//
// 该接口定义了最优控制问题中最小化目标的统一形式：
// - StageCost: 每个离散时刻的运行代价 l(x_k, u_k)；
// - TerminalCost: 终端代价 l_f(x_N)。
//
// 设计目的：
// - 让优化器（如 iLQR / AL-iLQR）与具体代价解耦；
// - 便于替换不同任务目标（跟踪、能耗、舒适性、约束软化等）。
class CostFunction {
 public:
  virtual ~CostFunction() = default;

  // 状态维度 n。
  virtual int StateDim() const = 0;

  // 控制维度 m。
  virtual int ControlDim() const = 0;

  // 阶段代价 l(x, u)：衡量当前时刻的状态偏差与控制代价。
  virtual double StageCost(const Vector& state, const Vector& control) const = 0;

  // 终端代价 l_f(x)：强化终点目标（位置、姿态、速度等）。
  virtual double TerminalCost(const Vector& state) const = 0;
};

}  // namespace my_al_ilqr
