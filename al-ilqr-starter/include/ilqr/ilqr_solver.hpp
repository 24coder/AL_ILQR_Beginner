#pragma once

#include <vector>

#include "core/trajectory.hpp"
#include "core/types.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

// iLQR 求解器的可调参数。
// 这些参数主要控制：
// 1. 外层迭代何时停止；
// 2. 数值求导时的扰动大小；
// 3. backward pass 中的正则化强度；
// 4. forward pass 中线搜索的步长衰减和接受条件。
struct ILQROptions {
  // 最大外层迭代次数。
  int max_iterations = 50;

  // 当相邻两次迭代的代价下降小于该阈值时，可认为已经收敛。
  double cost_tolerance = 1e-6;

  // 数值求导使用的有限差分扰动大小。
  double derivative_epsilon = 1e-4;

  // backward pass 初始正则化系数。
  // 正则化通常用于改善 Quu 的数值稳定性，避免矩阵接近奇异。
  double regularization_init = 1e-6;

  // 正则化允许的最小值。
  double regularization_min = 1e-8;

  // 正则化允许的最大值。
  // 若增长到该值仍无法完成 backward pass，通常说明当前轨迹局部模型较差。
  double regularization_max = 1e6;

  // backward pass 失败或步长不可接受时，正则化增大倍数。
  double regularization_increase_factor = 10.0;

  // 当前迭代效果较好时，正则化减小倍数。
  double regularization_decrease_factor = 10.0;

  // 线搜索最多尝试多少个 alpha。
  int line_search_max_iterations = 8;

  // 线搜索步长缩小系数。
  // 例如从 1.0 开始，随后依次尝试 0.5、0.25、0.125 ...
  double line_search_decrease_factor = 0.5;

  // 接受步长时，改进比率的下界。
  // 一般用于判断“实际下降”是否至少与“模型预测下降”同量级。
  double line_search_accept_lower = 1e-4;

  // 接受步长时，改进比率的上界。
  // 过大的比率通常意味着局部二次模型不够可信或数值异常。
  double line_search_accept_upper = 10.0;
};

// iLQR（iterative Linear Quadratic Regulator）求解器。
//
// 目标：
// 在给定初始状态和一组初始控制序列的基础上，迭代优化整条轨迹，
// 使系统动力学约束下的总成本尽可能小。
//
// 典型流程：
// 1. 用当前轨迹线性化动力学、二次近似代价；
// 2. backward pass 计算反馈增益 K 和前馈项 d；
// 3. forward pass 用线搜索生成新轨迹并判断是否接受；
// 4. 重复直到收敛或达到最大迭代次数。
class ILQRSolver {
 public:
  // problem: 被求解的最优控制问题，包含动力学、阶段代价、终端代价等。
  // options: 求解器超参数，控制收敛和数值稳定性。
  explicit ILQRSolver(const OptimalControlProblem& problem, ILQROptions options = {});

  // 每次外层迭代对应的总成本历史。
  const std::vector<double>& CostHistory() const { return cost_history_; }

  // 每次成功 forward pass 接受的线搜索步长 alpha。
  const std::vector<double>& AlphaHistory() const { return alpha_history_; }

  // 每次迭代使用的正则化参数历史。
  const std::vector<double>& RegularizationHistory() const { return regularization_history_; }

  // 每次迭代的改进比率历史。
  // 常见定义为：实际下降 / 预测下降。
  const std::vector<double>& ImprovementRatioHistory() const { return improvement_ratio_history_; }

  // backward pass 计算得到的反馈增益 K_k。
  // 一般用于控制律：u_k = u_k_nominal + alpha * d_k + K_k (x_k - x_k_nominal)
  const std::vector<Matrix>& FeedbackGains() const { return feedback_gains_; }

  // backward pass 计算得到的前馈项 d_k。
  const std::vector<Vector>& FeedforwardTerms() const { return feedforward_terms_; }

  // 从初始状态和初始控制序列出发求解最优轨迹。
  //
  // initial_state: 初始状态 x_0。
  // initial_controls: 初始控制猜测 u_0 ... u_{N-1}。
  //
  // 返回值：
  // 优化后的轨迹，包含整条状态序列和控制序列。
  Trajectory Solve(const Vector& initial_state, const std::vector<Vector>& initial_controls);

 private:
  // 某一阶段代价 l(x, u) 的二次展开结果：
  // l(x, u) ≈ const + lx^T dx + lu^T du
  //           + 1/2 dx^T lxx dx + du^T lux dx + 1/2 du^T luu du
  struct CostExpansion {
    // 对状态的一阶导数 ∂l/∂x。
    Vector lx;

    // 对控制的一阶导数 ∂l/∂u。
    Vector lu;

    // 对状态的二阶导数 ∂²l/∂x²。
    Matrix lxx;

    // 对控制和状态的混合二阶导数 ∂²l/∂u∂x。
    Matrix lux;

    // 对控制的二阶导数 ∂²l/∂u²。
    Matrix luu;
  };

  // 根据问题维度和时域长度，确保内部工作区容器已经分配到正确大小。
  void EnsureWorkspaceSizes();

  // 在当前轨迹上更新：
  // 1. 动力学雅可比 A_k = ∂f/∂x；
  // 2. 动力学雅可比 B_k = ∂f/∂u；
  // 3. 每个阶段的代价二次展开。
  void UpdateExpansions(const Trajectory& trajectory);

  // backward pass：
  // 从终端时刻向前递推局部 value function，计算每一步的反馈增益和前馈项。
  //
  // regularization: 当前 Levenberg-Marquardt 风格正则化系数。
  // expected_linear / expected_quadratic:
  // 输出模型预测的下降量的一次项和二次项，用于 forward pass 计算改进比率。
  //
  // 返回值：
  // true 表示 backward pass 成功；
  // false 通常表示某一步的 Quu 数值不稳定或不可逆，需要增大正则化。
  bool BackwardPass(const Trajectory& trajectory,
                    double regularization,
                    double* expected_linear,
                    double* expected_quadratic);

  // forward pass：
  // 使用当前名义轨迹、反馈增益和前馈项，对多个 alpha 进行线搜索，
  // 生成候选轨迹并选择满足接受条件的结果。
  //
  // initial_state: 固定起点 x_0。
  // nominal_trajectory: 当前被线性化/二次近似的名义轨迹。
  // current_cost: 名义轨迹当前总成本。
  // expected_linear / expected_quadratic: backward pass 给出的预测下降模型。
  // accepted_trajectory / accepted_cost / accepted_alpha / accepted_ratio:
  // 若找到可接受步长，则写入对应结果。
  //
  // 返回值：
  // true 表示线搜索成功找到可接受的新轨迹；false 表示本轮 forward pass 失败。
  bool ForwardPass(const Vector& initial_state,
                   const Trajectory& nominal_trajectory,
                   double current_cost,
                   double expected_linear,
                   double expected_quadratic,
                   Trajectory* accepted_trajectory,
                   double* accepted_cost,
                   double* accepted_alpha,
                   double* accepted_ratio);

  // 在给定 (x, u) 处数值计算动力学对状态的雅可比 A = ∂f/∂x。
  Matrix DynamicsJacobianState(const Vector& state, const Vector& control) const;

  // 在给定 (x, u) 处数值计算动力学对控制的雅可比 B = ∂f/∂u。
  Matrix DynamicsJacobianControl(const Vector& state, const Vector& control) const;

  // 计算第 k 个阶段代价对 (x_k, u_k) 的二次展开。
  CostExpansion StageCostExpansion(int k, const Vector& state, const Vector& control) const;

  // 计算终端代价对 x_N 的一阶、二阶展开。
  // 返回值分别是：
  // 1. 终端代价梯度；
  // 2. 终端代价 Hessian。
  std::pair<Vector, Matrix> TerminalCostExpansion(const Vector& state) const;

  // 计算整条轨迹的总成本（阶段成本累加 + 终端成本）。
  double EvaluateTrajectoryCost(const Trajectory& trajectory) const;

  // 被求解的最优控制问题定义。
  const OptimalControlProblem& problem_;

  // 求解器配置参数。
  ILQROptions options_;

  // 线性化动力学中的 A_k = ∂f/∂x。
  std::vector<Matrix> A_;

  // 线性化动力学中的 B_k = ∂f/∂u。
  std::vector<Matrix> B_;

  // 每个阶段代价的二次展开缓存。
  std::vector<CostExpansion> stage_expansions_;

  // 每一步的反馈增益 K_k。
  std::vector<Matrix> feedback_gains_;

  // 每一步的前馈修正 d_k。
  std::vector<Vector> feedforward_terms_;

  // 迭代中的总成本历史。
  std::vector<double> cost_history_;

  // 每次成功更新时采用的 alpha 历史。
  std::vector<double> alpha_history_;

  // 每次迭代对应的正则化参数历史。
  std::vector<double> regularization_history_;

  // 每次迭代的改进比率历史。
  std::vector<double> improvement_ratio_history_;
};

}  // namespace my_al_ilqr
