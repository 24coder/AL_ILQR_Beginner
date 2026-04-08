#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "constraints/control_box_constraint.hpp"
#include "constraints/terminal_goal_constraint.hpp"
#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"
#include "debug/debug_console_snapshot.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  // Phase 7：完整 AL-iLQR（Augmented Lagrangian + iLQR）示例。
  //
  // 这个示例演示的核心思想是：
  // 1. 用 iLQR 处理“给定代价函数”下的轨迹优化；
  // 2. 用增广拉格朗日（AL）在外层逐步处理约束；
  // 3. 即使初始控制序列明显违反约束，也希望通过外层 penalty / multiplier 更新，
  //    最终把解拉回到可行域附近，并得到一条较优轨迹。
  //
  // 对于这个例子：
  // - 系统动力学：独轮车模型（x, y, theta）
  // - 控制量：通常可理解为线速度 v 和角速度 w
  // - 约束：
  //   a) 每个时刻的控制输入都必须落在给定 box 范围内
  //   b) 终端状态需要满足目标约束（到达指定目标状态）
  using my_al_ilqr::ALILQROptions;
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::ConstrainedOptimalControlProblem;
  using my_al_ilqr::ControlBoxConstraint;
  using my_al_ilqr::ILQROptions;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::TerminalGoalConstraint;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  // -----------------------------
  // 1. 构造系统动力学模型
  // -----------------------------
  // 使用独轮车模型作为被控对象。
  // 状态一般为 x = [px, py, theta]^T，
  // 控制一般为 u = [v, omega]^T。
  auto dynamics = std::make_shared<UnicycleModel>();

  // -----------------------------
  // 2. 构造二次型代价函数
  // -----------------------------
  // 代价函数通常由三部分组成：
  // - 运行状态代价   (x_k - x_ref)^T Q  (x_k - x_ref)
  // - 运行控制代价   (u_k - u_ref)^T R  (u_k - u_ref)
  // - 终端状态代价   (x_N - x_ref)^T Qf (x_N - x_ref)
  //
  // 这里：
  // - Q  控制中间过程对状态误差的惩罚强度
  // - R  控制中间过程对控制输入大小的惩罚强度
  // - Qf 控制最终时刻对目标状态偏差的惩罚强度
  Matrix Q = Matrix::Zero(3, 3);
  Q.diagonal() << 0.8, 0.8, 0.1;

  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.05, 0.03;

  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 20.0, 20.0, 2.0;

  // 目标状态：希望系统最终到达 (1.5, 0.6)，并且朝向接近 0。
  Vector x_ref(3);
  x_ref << 1.5, 0.6, 0.0;

  // 控制参考值：这里设为 0，表示默认不鼓励使用过大的控制输入。
  Vector u_ref = Vector::Zero(2);

  // 用 Q / R / Qf 和参考状态/控制构造标准二次代价。
  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);

  // 构造“无约束”的基础最优控制问题：
  // - horizon = 45：优化时域长度为 45 步
  // - dt = 0.1：离散时间步长为 0.1 秒
  // 因此总时域大约是 4.5 秒。
  auto base_problem = std::make_shared<OptimalControlProblem>(dynamics, cost, 45, 0.1);

  // -----------------------------
  // 3. 在基础问题上增加约束
  // -----------------------------
  // 将基础问题包装成“带约束”的最优控制问题，
  // 供外层 AL 求解器处理。
  ConstrainedOptimalControlProblem constrained_problem(base_problem);

  // 控制输入下界 / 上界。
  // 这定义了一个 2 维 box 约束：u_lb <= u_k <= u_ub。
  Vector u_lb(2);
  u_lb << -0.5, -0.35;

  Vector u_ub(2);
  u_ub << 0.5, 0.35;

  // 对每一个时刻 k 添加阶段约束（stage constraint）。
  // 这里的约束是控制量 box 约束：
  // - 第一个控制量必须在 [-0.5, 0.5]
  // - 第二个控制量必须在 [-0.35, 0.35]
  //
  // base_problem->StateDim() 传给约束对象，通常用于说明该约束
  // 所关联的问题状态维数，便于统一接口处理。
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    constrained_problem.AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(base_problem->StateDim(), u_lb, u_ub));
  }

  // 添加终端约束：要求最终状态接近/满足目标状态 x_ref。
  // 注意：这里既有终端代价 Qf，又有终端约束 TerminalGoalConstraint。
  // - 终端代价是“软偏好”，鼓励靠近目标；
  // - 终端约束是“硬要求”，由 AL 在外层推动满足。
  constrained_problem.AddTerminalConstraint(std::make_shared<TerminalGoalConstraint>(x_ref));

  // -----------------------------
  // 4. 设置初始状态和初始控制猜测
  // -----------------------------
  // 系统初始状态。
  // 位置从原点出发，初始航向稍微偏转 0.15 rad。
  Vector x0(3);
  x0 << 0.0, 0.0, 0.15;

  // 为求解器提供一组初始控制序列作为 warm start。
  // 大多数轨迹优化方法都对初值敏感，给出初始猜测有助于收敛。
  std::vector<Vector> initial_controls(base_problem->Horizon(), Vector::Zero(2));

  for (int k = 0; k < base_problem->Horizon(); ++k) {
    // 故意构造一个“较差”的初始控制：
    // - 第一个控制量 0.65 超过了上界 0.5
    // - 前 15 步第二个控制量 0.38 超过了上界 0.35
    //
    // 这么做是为了演示：
    // 即使初始猜测明显不可行，外层增广拉格朗日也会通过增大罚参数、
    // 调整乘子等方式，逐步把优化结果拉回约束可行域。
    initial_controls[k](0) = 0.65;
    initial_controls[k](1) = (k < 15) ? 0.38 : -0.12;
  }

  // 打印当前示例的关键配置快照，便于调试：
  // 通常会输出代价矩阵、参考状态、初始状态以及初始控制猜测等信息。
  my_al_ilqr::debug::PrintPhase7SetupSnapshot(Q, R, Qf, x_ref, u_ref, x0, initial_controls);

  // -----------------------------
  // 5. 配置内层 iLQR 参数
  // -----------------------------
  // 内层 iLQR 负责：在当前增广代价下，求解一轮局部轨迹优化。
  ILQROptions inner_options;

  // 单次内层求解允许的最大迭代次数。
  inner_options.max_iterations = 30;

  // 若代价下降小于该阈值，可认为内层已基本收敛。
  inner_options.cost_tolerance = 1e-5;

  // 数值求导时使用的扰动步长。
  inner_options.derivative_epsilon = 1e-4;

  // Levenberg-Marquardt 风格正则化初值。
  // 用于增强 backward pass 数值稳定性。
  inner_options.regularization_init = 1e-5;

  // 正则化最小/最大限制，避免过小导致不稳定、过大导致搜索停滞。
  inner_options.regularization_min = 1e-8;
  inner_options.regularization_max = 1e5;

  // 当 backward pass 或 line search 表现不佳时，增大正则化；
  // 当优化进展顺利时，减小正则化。
  inner_options.regularization_increase_factor = 10.0;
  inner_options.regularization_decrease_factor = 5.0;

  // 线搜索参数：
  // iLQR 计算出控制修正后，不一定直接整步接受，
  // 往往会尝试 alpha = 1, 0.5, 0.25 ... 这样的缩放，
  // 选择一个能带来合理代价下降的步长。
  inner_options.line_search_max_iterations = 10;
  inner_options.line_search_decrease_factor = 0.5;

  // 接受步长时对“实际下降 / 预测下降”比例的阈值范围。
  inner_options.line_search_accept_lower = 1e-4;
  inner_options.line_search_accept_upper = 10.0;

  // -----------------------------
  // 6. 配置外层 AL 参数
  // -----------------------------
  // 外层 AL 负责处理约束满足问题：
  // 每一轮会基于当前 penalty / multiplier 构造增广代价，
  // 然后调用一次内层 iLQR 进行优化，再根据约束违反情况更新外层参数。
  ALILQROptions options;
  options.inner_options = inner_options;

  // 重要外层参数说明：
  // - max_outer_iterations：AL 外循环次数上限；
  // - constraint_tolerance：判断“约束是否基本满足”的阈值；
  // - initial_penalty：初始罚参数；
  // - penalty_scaling：当违反下降不够理想时，罚参数的放大倍数；
  // - penalty_update_ratio：决定“违反是否下降足够多”的判断标准；
  // - max_penalty：罚参数上限，避免无限增大。
  options.max_outer_iterations = 8;
  options.constraint_tolerance = 2e-2;
  options.initial_penalty = 10.0;
  options.penalty_scaling = 3.0;
  options.penalty_update_ratio = 0.8;
  options.max_penalty = 1e6;

  // -----------------------------
  // 7. 创建求解器并执行优化
  // -----------------------------
  // ALILQRSolver：
  // - 外层做增广拉格朗日更新
  // - 内层调用 iLQR 求解当前增广问题
  ALILQRSolver solver(constrained_problem, options);

  // 以初始状态 x0 和初始控制猜测 initial_controls 为输入开始求解。
  // 返回结果中通常包括：
  // - 是否收敛
  // - 最终轨迹
  // - 每次外层迭代的日志（代价、违反度、罚参数等）
  const auto result = solver.Solve(x0, initial_controls);

  // -----------------------------
  // 8. 保存轨迹结果
  // -----------------------------
  // 将最终轨迹写入 CSV，方便后续画图或离线分析。
  const std::filesystem::path csv_path = "build/phase7_al_ilqr_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, result.trajectory);

  // -----------------------------
  // 9. 在控制台输出求解摘要
  // -----------------------------
  std::cout << "Phase 7 AL-iLQR demo\n";

  // 是否成功达到算法设定的收敛标准。
  std::cout << "converged = " << (result.converged ? "true" : "false") << "\n";

  // 实际进行了多少轮外层 AL 迭代。
  std::cout << "outer iterations = " << result.outer_logs.size() << "\n";

  // 如果存在外层日志，则输出最初和最终的约束违反程度。
  // 一般来说我们希望看到 final violation 明显小于 initial violation。
  if (!result.outer_logs.empty()) {
    std::cout << "initial logged violation = " << result.outer_logs.front().max_violation << "\n";
    std::cout << "final logged violation = " << result.outer_logs.back().max_violation << "\n";
  }

  // 输出终端状态，方便直接查看最终是否接近目标 x_ref。
  std::cout << "terminal state = " << result.trajectory.State(base_problem->Horizon()).transpose()
            << "\n";

  // 输出 CSV 路径，便于用户进一步用 Python / MATLAB / Excel 可视化。
  std::cout << "trajectory csv = " << csv_path << "\n";

  // 打印每次外层迭代的摘要：
  // - base_cost：原始代价
  // - aug_cost：增广拉格朗日后的代价
  // - violation：当前最大约束违反
  // - penalty：当前使用的最大罚参数
  // - inner_iters：本轮外层调用的内层 iLQR 迭代次数
  std::cout << "outer iteration summary:\n";
  for (const auto& log : result.outer_logs) {
    std::cout << "  outer " << log.outer_iteration << ": base_cost=" << log.base_cost
              << ", aug_cost=" << log.augmented_cost
              << ", violation=" << log.max_violation
              << ", penalty=" << log.max_penalty
              << ", inner_iters=" << log.inner_iterations << "\n";
  }

  return 0;
}
