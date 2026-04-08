#include "core/trajectory.hpp"

#include <stdexcept>

namespace my_al_ilqr {

// 构造轨迹对象并一次性完成内存布局：
// - states_ 大小为 horizon + 1；
// - controls_ 大小为 horizon。
// 这样后续 Rollout / 求解器迭代只做覆写，不需要反复扩容。
Trajectory::Trajectory(int state_dim, int control_dim, int horizon)
    : state_dim_(state_dim),
      control_dim_(control_dim),
      horizon_(horizon),
      states_(horizon + 1, Vector::Zero(state_dim)),
      controls_(horizon, Vector::Zero(control_dim)) {
  if (state_dim <= 0 || control_dim <= 0 || horizon <= 0) {
    throw std::invalid_argument("Trajectory dimensions and horizon must be positive.");
  }
}

// 返回状态变量在第 k 个离散时刻的状态向量的可写引用。
// 这里的“第 k 个状态”更准确地说是轨迹上的 x_k，而不是状态向量内部的第 k 个分量。
// 使用 at(k) 做边界检查，越界会抛异常，避免静默内存错误。
Vector& Trajectory::State(int k) {
  return states_.at(k);
}

// 返回状态变量在第 k 个离散时刻的状态向量的只读引用。
const Vector& Trajectory::State(int k) const {
  return states_.at(k);
}

// 返回第 k 个控制的可写引用。
Vector& Trajectory::Control(int k) {
  return controls_.at(k);
}

// 返回第 k 个控制的只读引用。
const Vector& Trajectory::Control(int k) const {
  return controls_.at(k);
}

}  // namespace my_al_ilqr
