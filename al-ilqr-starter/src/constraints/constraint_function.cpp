#include "constraints/constraint_function.hpp"

#include <algorithm>
#include <cmath>

namespace my_al_ilqr {

// 将约束向量聚合为单个“最大违反值”，用于外层罚函数与收敛判据。
// 数学含义：
// - 等式约束 g(x,u)=0: violation = max_i |g_i|
// - 不等式约束 h(x,u)<=0: violation = max(0, max_i h_i)
// 其中 h_i<=0 代表满足约束，h_i>0 代表违反约束。
double MaxViolation(const ConstraintFunction& constraint, const Vector& values) {
  if (values.size() == 0) {
    return 0.0;
  }

  if (constraint.Type() == ConstraintType::kEquality) {
    // 等式约束关心偏离 0 的绝对值。
    return values.cwiseAbs().maxCoeff();
  }

  // 不等式约束只关心正向“超限”部分。
  double max_violation = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    max_violation = std::max(max_violation, values(i));
  }
  return std::max(0.0, max_violation);
}

}  // namespace my_al_ilqr
