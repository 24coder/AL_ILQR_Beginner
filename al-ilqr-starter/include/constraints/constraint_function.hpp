#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

// 约束类型：
// - kEquality: 等式约束，目标是 g(x, u) = 0。
// - kInequality: 不等式约束，统一写成 h(x, u) <= 0 的形式。
enum class ConstraintType {
  kEquality,
  kInequality,
};

// 约束函数抽象接口。
// 求解器会在每个时刻(knot point)调用 Evaluate，得到约束值向量，
// 并基于约束类型计算违反程度与罚项。
class ConstraintFunction {
 public:
  virtual ~ConstraintFunction() = default;

  // 约束类别（等式 / 不等式）。
  virtual ConstraintType Type() const = 0;
  // 输入状态维度 x 的长度。
  virtual int StateDim() const = 0;
  // 输入控制维度 u 的长度；终端约束可为 0。
  virtual int ControlDim() const = 0;
  // 输出约束向量维度 m（即 g/h 的分量数）。
  virtual int OutputDim() const = 0;
  // 约束名字，用于日志、诊断与可视化。
  virtual std::string Name() const = 0;
  // 计算约束值。
  // 数据流：求解器提供 (state, control) -> 约束返回 values。
  virtual Vector Evaluate(const Vector& state, const Vector& control) const = 0;
};

using ConstraintPtr = std::shared_ptr<ConstraintFunction>;

// 一次约束评估的快照，通常用于调试和收敛分析。
struct ConstraintEvaluation {
  std::string name;
  ConstraintType type = ConstraintType::kEquality;
  int knot_point = 0;  // 时间离散索引 k
  Vector values;       // Evaluate 的原始输出
  double max_violation = 0.0;  // 聚合后的最大违反值
};

// 计算约束向量的最大违反程度。
// 等式约束采用 max |g_i|；不等式约束采用 max(0, max h_i)。
double MaxViolation(const ConstraintFunction& constraint, const Vector& values);

}  // namespace my_al_ilqr
