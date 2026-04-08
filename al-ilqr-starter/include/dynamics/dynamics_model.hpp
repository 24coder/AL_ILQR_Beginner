#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

// 离散时间动力学模型抽象接口。
// 数据流：给定当前状态 x_k、控制 u_k 和步长 dt，返回下一状态 x_{k+1}。
class DynamicsModel {
 public:
  virtual ~DynamicsModel() = default;

  // 状态向量维度 n。
  virtual int StateDim() const = 0;
  // 控制向量维度 m。
  virtual int ControlDim() const = 0;
  // 一步状态转移函数：x_{k+1} = f(x_k, u_k, dt)。
  virtual Vector NextState(const Vector& state, const Vector& control, double dt) const = 0;
};

}  // namespace my_al_ilqr
