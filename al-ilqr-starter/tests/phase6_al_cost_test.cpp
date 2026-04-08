#include <cassert>
#include <memory>
#include <vector>

#include "al/augmented_lagrangian_cost.hpp"
#include "constraints/control_box_constraint.hpp"
#include "constraints/terminal_goal_constraint.hpp"
#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::AugmentedLagrangianProblem;
  using my_al_ilqr::ConstrainedOptimalControlProblem;
  using my_al_ilqr::ControlBoxConstraint;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::TerminalGoalConstraint;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  // 本测试验证增广拉格朗日（AL）代价层的关键行为：
  // 1) 在存在约束违反时，AL 阶段代价应高于原始阶段代价；
  // 2) 调用 UpdateDuals 后应出现正的对偶变量（lambda）；
  // 3) 调用 UpdatePenalties 后最大罚因子应增大。
  // 这些性质共同保证 AL 外层更新机制确实在“惩罚约束违反”。
  auto dynamics = std::make_shared<UnicycleModel>();
  Matrix Q = Matrix::Identity(3, 3);
  Matrix R = Matrix::Identity(2, 2);
  Matrix Qf = 2.0 * Matrix::Identity(3, 3);
  Vector x_ref = Vector::Zero(3);
  Vector u_ref = Vector::Zero(2);
  auto base_cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  auto base_problem = std::make_shared<OptimalControlProblem>(dynamics, base_cost, 4, 0.1);

  ConstrainedOptimalControlProblem constrained_problem(base_problem);
  Vector u_lb(2);
  u_lb << -0.5, -0.2;
  Vector u_ub(2);
  u_ub << 0.5, 0.2;
  constrained_problem.AddStageConstraint(
      0, std::make_shared<ControlBoxConstraint>(base_problem->StateDim(), u_lb, u_ub));
  constrained_problem.AddTerminalConstraint(std::make_shared<TerminalGoalConstraint>(x_ref));

  AugmentedLagrangianProblem al_problem(constrained_problem, 10.0, 4.0);

  Vector x0 = Vector::Zero(3);
  std::vector<Vector> controls(base_problem->Horizon(), Vector::Zero(2));
  controls[0](0) = 0.8;
  controls[0](1) = 0.0;

  const auto trajectory = base_problem->Rollout(x0, controls);
  const double base_stage_cost =
      base_problem->StageCostFunction(0).StageCost(trajectory.State(0), trajectory.Control(0));
  const double al_stage_cost =
      al_problem.StageCost(0)->StageCost(trajectory.State(0), trajectory.Control(0));

  // 违反约束时，增广代价应上浮。
  assert(al_stage_cost > base_stage_cost);

  const double penalty_before = al_problem.MaxPenalty();
  al_problem.UpdateDuals(trajectory);

  // 检查对偶变量是否被有效更新为正值（至少一项）。
  bool has_positive_lambda = false;
  for (const auto& data : al_problem.StageCost(0)->Constraints()) {
    if (data.lambda.maxCoeff() > 0.0) {
      has_positive_lambda = true;
    }
  }
  assert(has_positive_lambda);

  // 检查罚因子更新逻辑是否生效。
  al_problem.UpdatePenalties();
  assert(al_problem.MaxPenalty() > penalty_before);

  return 0;
}
