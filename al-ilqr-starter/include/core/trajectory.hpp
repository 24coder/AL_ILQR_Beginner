#pragma once

#include <iomanip>
#include <iostream>
#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

// 轨迹容器：按离散时域存储状态与控制。
// 约定 horizon = N 时：
// - 状态序列有 N+1 个点：x_0 ... x_N
// - 控制序列有 N 个点：u_0 ... u_{N-1}
class Trajectory {
 public:
  // 构造并按维度预分配所有时刻的状态/控制向量。
  // 每个状态初值为 Zero(state_dim)，每个控制初值为 Zero(control_dim)。
  Trajectory(int state_dim, int control_dim, int horizon);

  // 基本维度信息。
  int StateDim() const { return state_dim_; }
  int ControlDim() const { return control_dim_; }
  int Horizon() const { return horizon_; }

  // 设置/读取轨迹离散时间步长，仅用于记录与下游读取。
  void SetTimeStep(double dt) { dt_ = dt; }
  double TimeStep() const { return dt_; }

  // 访问第 k 个状态（0 <= k <= N）。
  Vector& State(int k);
  const Vector& State(int k) const;

  // 访问第 k 个控制（0 <= k < N）。
  Vector& Control(int k);
  const Vector& Control(int k) const;

#ifndef NDEBUG
  // 调试用：打印所有状态和控制到终端，每列固定宽度对齐。
  void Print() const {
    Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, "  ", "", "", "", "", "");
    std::cout << "=== Trajectory ===" << std::endl;
    std::cout << "states_ (size=" << states_.size() << "):" << std::endl;
    for (int k = 0; k < static_cast<int>(states_.size()); ++k) {
      std::cout << "  x[" << std::setw(3) << k << "] = ";
      for (int i = 0; i < states_[k].size(); ++i) {
        std::cout << std::setw(12) << std::setprecision(6) << std::fixed << states_[k](i);
      }
      std::cout << std::endl;
    }
    std::cout << "controls_ (size=" << controls_.size() << "):" << std::endl;
    for (int k = 0; k < static_cast<int>(controls_.size()); ++k) {
      std::cout << "  u[" << std::setw(3) << k << "] = ";
      for (int i = 0; i < controls_[k].size(); ++i) {
        std::cout << std::setw(12) << std::setprecision(6) << std::fixed << controls_[k](i);
      }
      std::cout << std::endl;
    }
    std::cout << "==================" << std::endl;
  }
#endif

 private:
  // 状态维度 nx。
  int state_dim_;

  // 控制维度 nu。
  int control_dim_;

  // 时域长度 N。
  int horizon_;

  // 时间步长 dt（可选记录项）。
  double dt_ = 0.0;

  // 状态缓存，长度为 N+1。
  std::vector<Vector> states_;

  // 控制缓存，长度为 N。
  std::vector<Vector> controls_;
};

}  // namespace my_al_ilqr
