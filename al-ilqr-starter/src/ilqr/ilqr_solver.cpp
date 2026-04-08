#include "ilqr/ilqr_solver.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_al_ilqr {

namespace {

// 读取第 k 个时刻的阶段代价 l_k(x_k, u_k)。
// 单独包一层的作用是：
// 1. 让后面的有限差分代码更紧凑；
// 2. 把“如何从 problem 中取出阶段代价对象”这个细节隐藏起来。
double StageCostAt(const OptimalControlProblem& problem, int k, const Vector& x, const Vector& u) {
  return problem.StageCostFunction(k).StageCost(x, u);
}

// 读取终端代价 l_f(x_N)。
// 与 StageCostAt 对应，用于终端梯度/Hessian 的数值计算。
double TerminalCostAt(const OptimalControlProblem& problem, const Vector& x) {
  return problem.TerminalCostFunction().TerminalCost(x);
}

// 用中心差分计算阶段代价对状态的梯度：lx = ∂l/∂x。
// 第 i 维梯度近似为：
//   (l(x + eps * e_i, u) - l(x - eps * e_i, u)) / (2 * eps)
// 中心差分比前向差分更对称，通常数值精度更好。
Vector CentralDifferenceGradientState(const OptimalControlProblem& problem,
                                      int k,
                                      const Vector& x,
                                      const Vector& u,
                                      double eps) {
  Vector grad = Vector::Zero(x.size());
  for (int i = 0; i < x.size(); ++i) {
    Vector x_plus = x;
    Vector x_minus = x;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    grad(i) = (StageCostAt(problem, k, x_plus, u) - StageCostAt(problem, k, x_minus, u)) /
              (2.0 * eps);
  }
  return grad;
}

// 用中心差分计算阶段代价对控制的梯度：lu = ∂l/∂u。
Vector CentralDifferenceGradientControl(const OptimalControlProblem& problem,
                                        int k,
                                        const Vector& x,
                                        const Vector& u,
                                        double eps) {
  Vector grad = Vector::Zero(u.size());
  for (int i = 0; i < u.size(); ++i) {
    Vector u_plus = u;
    Vector u_minus = u;
    u_plus(i) += eps;
    u_minus(i) -= eps;
    grad(i) = (StageCostAt(problem, k, x, u_plus) - StageCostAt(problem, k, x, u_minus)) /
              (2.0 * eps);
  }
  return grad;
}

// 用中心差分计算阶段代价对状态的 Hessian：lxx = ∂²l/∂x²。
// 这里对每一对 (i, j) 使用四点公式：
//   [l(x_i+, x_j+) - l(x_i+, x_j-) - l(x_i-, x_j+) + l(x_i-, x_j-)] / (4 eps²)
// 当 i == j 时，这个写法仍然成立，只是变成沿单个坐标方向的二阶差分。
Matrix CentralDifferenceHessianState(const OptimalControlProblem& problem,
                                     int k,
                                     const Vector& x,
                                     const Vector& u,
                                     double eps) {
  Matrix hess = Matrix::Zero(x.size(), x.size());
  for (int i = 0; i < x.size(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      Vector x_pp = x;
      Vector x_pm = x;
      Vector x_mp = x;
      Vector x_mm = x;
      x_pp(i) += eps;
      x_pp(j) += eps;
      x_pm(i) += eps;
      x_pm(j) -= eps;
      x_mp(i) -= eps;
      x_mp(j) += eps;
      x_mm(i) -= eps;
      x_mm(j) -= eps;
      hess(i, j) =
          (StageCostAt(problem, k, x_pp, u) - StageCostAt(problem, k, x_pm, u) -
           StageCostAt(problem, k, x_mp, u) + StageCostAt(problem, k, x_mm, u)) /
          (4.0 * eps * eps);
    }
  }
  return hess;
}

// 用中心差分计算阶段代价对控制的 Hessian：luu = ∂²l/∂u²。
// 逻辑与状态 Hessian 完全一致，只是扰动变量从 x 换成了 u。
Matrix CentralDifferenceHessianControl(const OptimalControlProblem& problem,
                                       int k,
                                       const Vector& x,
                                       const Vector& u,
                                       double eps) {
  Matrix hess = Matrix::Zero(u.size(), u.size());
  for (int i = 0; i < u.size(); ++i) {
    for (int j = 0; j < u.size(); ++j) {
      Vector u_pp = u;
      Vector u_pm = u;
      Vector u_mp = u;
      Vector u_mm = u;
      u_pp(i) += eps;
      u_pp(j) += eps;
      u_pm(i) += eps;
      u_pm(j) -= eps;
      u_mp(i) -= eps;
      u_mp(j) += eps;
      u_mm(i) -= eps;
      u_mm(j) -= eps;
      hess(i, j) =
          (StageCostAt(problem, k, x, u_pp) - StageCostAt(problem, k, x, u_pm) -
           StageCostAt(problem, k, x, u_mp) + StageCostAt(problem, k, x, u_mm)) /
          (4.0 * eps * eps);
    }
  }
  return hess;
}

// 用中心差分计算阶段代价的混合二阶导：lux = ∂²l/∂u∂x。
// 返回矩阵形状为 (nu, nx)，即：第 i 行第 j 列表示对 u_i 再对 x_j 的二阶导。
// 这正好匹配后续 backward pass 中 Qux 的维度约定。
Matrix CentralDifferenceHessianCross(const OptimalControlProblem& problem,
                                     int k,
                                     const Vector& x,
                                     const Vector& u,
                                     double eps) {
  Matrix hess = Matrix::Zero(u.size(), x.size());
  for (int i = 0; i < u.size(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      Vector x_plus = x;
      Vector x_minus = x;
      Vector u_plus = u;
      Vector u_minus = u;
      x_plus(j) += eps;
      x_minus(j) -= eps;
      u_plus(i) += eps;
      u_minus(i) -= eps;
      hess(i, j) =
          (StageCostAt(problem, k, x_plus, u_plus) - StageCostAt(problem, k, x_plus, u_minus) -
           StageCostAt(problem, k, x_minus, u_plus) + StageCostAt(problem, k, x_minus, u_minus)) /
          (4.0 * eps * eps);
    }
  }
  return hess;
}

// 用中心差分计算终端代价对状态的梯度：∂l_f/∂x_N。
Vector CentralDifferenceTerminalGradient(const OptimalControlProblem& problem,
                                         const Vector& x,
                                         double eps) {
  Vector grad = Vector::Zero(x.size());
  for (int i = 0; i < x.size(); ++i) {
    Vector x_plus = x;
    Vector x_minus = x;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    grad(i) = (TerminalCostAt(problem, x_plus) - TerminalCostAt(problem, x_minus)) / (2.0 * eps);
  }
  return grad;
}

// 用中心差分计算终端代价对状态的 Hessian：∂²l_f/∂x_N²。
Matrix CentralDifferenceTerminalHessian(const OptimalControlProblem& problem,
                                        const Vector& x,
                                        double eps) {
  Matrix hess = Matrix::Zero(x.size(), x.size());
  for (int i = 0; i < x.size(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      Vector x_pp = x;
      Vector x_pm = x;
      Vector x_mp = x;
      Vector x_mm = x;
      x_pp(i) += eps;
      x_pp(j) += eps;
      x_pm(i) += eps;
      x_pm(j) -= eps;
      x_mp(i) -= eps;
      x_mp(j) += eps;
      x_mm(i) -= eps;
      x_mm(j) -= eps;
      hess(i, j) = (TerminalCostAt(problem, x_pp) - TerminalCostAt(problem, x_pm) -
                    TerminalCostAt(problem, x_mp) + TerminalCostAt(problem, x_mm)) /
                   (4.0 * eps * eps);
    }
  }
  return hess;
}

}  // namespace

// 构造求解器时立即分配内部工作区，避免后续迭代中重复按尺寸构造缓存容器。
ILQRSolver::ILQRSolver(const OptimalControlProblem& problem, ILQROptions options)
    : problem_(problem), options_(std::move(options)) {
  EnsureWorkspaceSizes();
}

// 按时域长度 N、状态维度 nx、控制维度 nu 初始化所有缓存。
// 这些缓存会在每轮迭代中被反复覆写：
// - A_[k], B_[k]：当前名义轨迹上的动力学线性化；
// - stage_expansions_[k]：当前名义轨迹上的代价二次展开；
// - feedback_gains_[k], feedforward_terms_[k]：backward pass 的结果。
void ILQRSolver::EnsureWorkspaceSizes() {
  const int N = problem_.Horizon();
  const int nx = problem_.StateDim();
  const int nu = problem_.ControlDim();
  A_.assign(N, Matrix::Zero(nx, nx));
  B_.assign(N, Matrix::Zero(nx, nu));
  stage_expansions_.assign(N, CostExpansion{
                                  Vector::Zero(nx),
                                  Vector::Zero(nu),
                                  Matrix::Zero(nx, nx),
                                  Matrix::Zero(nu, nx),
                                  Matrix::Zero(nu, nu),
                              });
  feedback_gains_.assign(N, Matrix::Zero(nu, nx));
  feedforward_terms_.assign(N, Vector::Zero(nu));
}

// 对离散动力学 x_{k+1} = f(x_k, u_k) 在给定点处对状态做数值线性化。
// 返回 A = ∂f/∂x，维度为 (nx, nx)。
// 第 i 列表示：状态第 i 维发生微小变化时，下一状态各维如何变化。
Matrix ILQRSolver::DynamicsJacobianState(const Vector& state, const Vector& control) const {
  const double eps = options_.derivative_epsilon;
  Matrix jac = Matrix::Zero(problem_.StateDim(), problem_.StateDim());
  for (int i = 0; i < problem_.StateDim(); ++i) {
    Vector x_plus = state;
    Vector x_minus = state;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    const Vector f_plus = problem_.Dynamics().NextState(x_plus, control, problem_.TimeStep());
    const Vector f_minus = problem_.Dynamics().NextState(x_minus, control, problem_.TimeStep());
    jac.col(i) = (f_plus - f_minus) / (2.0 * eps);
  }
  return jac;
}

// 对离散动力学在给定点处对控制做数值线性化。
// 返回 B = ∂f/∂u，维度为 (nx, nu)。
// 第 i 列表示：控制第 i 维发生微小变化时，下一状态各维如何变化。
Matrix ILQRSolver::DynamicsJacobianControl(const Vector& state, const Vector& control) const {
  const double eps = options_.derivative_epsilon;
  Matrix jac = Matrix::Zero(problem_.StateDim(), problem_.ControlDim());
  for (int i = 0; i < problem_.ControlDim(); ++i) {
    Vector u_plus = control;
    Vector u_minus = control;
    u_plus(i) += eps;
    u_minus(i) -= eps;
    const Vector f_plus = problem_.Dynamics().NextState(state, u_plus, problem_.TimeStep());
    const Vector f_minus = problem_.Dynamics().NextState(state, u_minus, problem_.TimeStep());
    jac.col(i) = (f_plus - f_minus) / (2.0 * eps);
  }
  return jac;
}

// 在给定时刻 k、给定名义点 (x_k, u_k) 处，构造阶段代价的二次近似：
//   l(x, u) ≈ const + lx^T dx + lu^T du
//                 + 1/2 dx^T lxx dx + du^T lux dx + 1/2 du^T luu du
// iLQR 的 backward pass 需要的就是这一组一阶/二阶导信息。
ILQRSolver::CostExpansion ILQRSolver::StageCostExpansion(int k,
                                                         const Vector& state,
                                                         const Vector& control) const {
  const double eps = options_.derivative_epsilon;
  return CostExpansion{
      CentralDifferenceGradientState(problem_, k, state, control, eps),
      CentralDifferenceGradientControl(problem_, k, state, control, eps),
      CentralDifferenceHessianState(problem_, k, state, control, eps),
      CentralDifferenceHessianCross(problem_, k, state, control, eps),
      CentralDifferenceHessianControl(problem_, k, state, control, eps),
  };
}

// 终端代价只依赖终端状态 x_N，因此只需要返回：
// 1. 梯度 Vx 的初值；
// 2. Hessian Vxx 的初值。
// backward pass 就是从这个终点开始往前递推的。
std::pair<Vector, Matrix> ILQRSolver::TerminalCostExpansion(const Vector& state) const {
  const double eps = options_.derivative_epsilon;
  return {CentralDifferenceTerminalGradient(problem_, state, eps),
          CentralDifferenceTerminalHessian(problem_, state, eps)};
}

// 在整条当前轨迹上更新局部模型。
// 对于每个时刻 k：
// - A_[k], B_[k] 描述动力学的一阶近似；
// - stage_expansions_[k] 描述阶段代价的二阶近似。
// 这些量共同定义了一个“局部 LQR 子问题”。
void ILQRSolver::UpdateExpansions(const Trajectory& trajectory) {
  for (int k = 0; k < problem_.Horizon(); ++k) {
    A_[k] = DynamicsJacobianState(trajectory.State(k), trajectory.Control(k));
    B_[k] = DynamicsJacobianControl(trajectory.State(k), trajectory.Control(k));
    stage_expansions_[k] = StageCostExpansion(k, trajectory.State(k), trajectory.Control(k));
  }
}

// 计算整条轨迹的真实总代价，而不是局部近似代价。
// 线搜索阶段需要用它来判断候选轨迹是否真的更优。
double ILQRSolver::EvaluateTrajectoryCost(const Trajectory& trajectory) const {
  return problem_.TotalCost(trajectory);
}

// backward pass 是 iLQR 的核心之一。
// 它从终端时刻开始反向递推 value function 的局部二次近似：
//   V_k(x) ≈ const + Vx^T dx + 1/2 dx^T Vxx dx
// 并在每个时刻构造 Q 函数后解出局部控制律：
//   du_k = d_k + K_k dx_k
// 其中：
// - d_k = feedforward_terms_[k] 是前馈修正；
// - K_k = feedback_gains_[k] 是反馈增益。
//
// 若 Quu_reg 不是正定矩阵，LLT 分解会失败，此时说明当前局部二次模型不够稳定，
// 调用方会提高 regularization 后重试。
bool ILQRSolver::BackwardPass(const Trajectory& trajectory,
                              double regularization,
                              double* expected_linear,
                              double* expected_quadratic) {
  // 终端 value function 直接来自终端代价的梯度和 Hessian。
  const auto [terminal_gradient, terminal_hessian] =
      TerminalCostExpansion(trajectory.State(problem_.Horizon()));
  Vector Vx = terminal_gradient;
  Matrix Vxx = terminal_hessian;
  *expected_linear = 0.0;
  *expected_quadratic = 0.0;

  for (int k = problem_.Horizon() - 1; k >= 0; --k) {
    const auto& expansion = stage_expansions_[k];
    const Matrix& A = A_[k];
    const Matrix& B = B_[k];

    // Q 函数是“当前一步代价 + 下一步 value function”在局部线性/二次模型下的组合。
    // 这些公式是标准 iLQR / DDP 推导中的核心结果。
    const Vector Qx = expansion.lx + A.transpose() * Vx;
    const Vector Qu = expansion.lu + B.transpose() * Vx;
    const Matrix Qxx = expansion.lxx + A.transpose() * Vxx * A;
    const Matrix Qux = expansion.lux + B.transpose() * Vxx * A;
    const Matrix Quu = expansion.luu + B.transpose() * Vxx * B;

    // 给 Quu 加上 LM 风格正则化，改善数值稳定性。
    // 从直觉上看，相当于“抑制控制更新过大”。
    const Matrix Quu_reg =
        Quu + regularization * Matrix::Identity(problem_.ControlDim(), problem_.ControlDim());

    Eigen::LLT<Matrix> llt(Quu_reg);
    if (llt.info() != Eigen::Success) {
      return false;
    }

    // 最优局部控制律：
    //   K = -Quu^{-1} Qux
    //   d = -Quu^{-1} Qu
    // 这里用 LLT 解线性方程，避免显式求逆。
    feedback_gains_[k] = -llt.solve(Qux);
    feedforward_terms_[k] = -llt.solve(Qu);

    // 记录模型预测下降量：
    // ΔJ_hat(α) ≈ -(α * linear + α² * quadratic)
    // forward pass 会用它与实际下降量做比值，判断局部模型是否可信。
    *expected_linear += Qu.dot(feedforward_terms_[k]);
    *expected_quadratic += 0.5 * feedforward_terms_[k].dot(Quu * feedforward_terms_[k]);

    // 把当前时刻消元后，更新前一时刻需要用到的 value function 展开。
    // 这是动态规划递推的关键步骤。
    Vx = Qx + feedback_gains_[k].transpose() * Quu * feedforward_terms_[k] +
         feedback_gains_[k].transpose() * Qu + Qux.transpose() * feedforward_terms_[k];
    Vxx = Qxx + feedback_gains_[k].transpose() * Quu * feedback_gains_[k] +
          feedback_gains_[k].transpose() * Qux + Qux.transpose() * feedback_gains_[k];

    // 数值误差可能让 Vxx 轻微不对称，这里强制对称化，避免误差继续向前传播。
    Vxx = 0.5 * (Vxx + Vxx.transpose());
  }

  return true;
}

// forward pass 根据 backward pass 算出的 (d_k, K_k) 真正 rollout 一条新轨迹。
// 使用的控制律是：
//   u_k_new = u_k_nominal + α d_k + K_k (x_k_new - x_k_nominal)
// 其中：
// - α 用于线搜索，控制前馈步长大小；
// - 反馈项 K_k(...) 用于修正新轨迹与名义轨迹的状态偏差。
//
// 对每个候选 α，都会重新通过真实动力学展开整条轨迹，再用真实代价做验收。
bool ILQRSolver::ForwardPass(const Vector& initial_state,
                             const Trajectory& nominal_trajectory,
                             double current_cost,
                             double expected_linear,
                             double expected_quadratic,
                             Trajectory* accepted_trajectory,
                             double* accepted_cost,
                             double* accepted_alpha,
                             double* accepted_ratio) {
  double alpha = 1.0;

  for (int iter = 0; iter < options_.line_search_max_iterations; ++iter) {
    Trajectory candidate(problem_.StateDim(), problem_.ControlDim(), problem_.Horizon());
    candidate.SetTimeStep(problem_.TimeStep());
    candidate.State(0) = initial_state;

    for (int k = 0; k < problem_.Horizon(); ++k) {
      const Vector dx = candidate.State(k) - nominal_trajectory.State(k);
      candidate.Control(k) =
          nominal_trajectory.Control(k) + alpha * feedforward_terms_[k] + feedback_gains_[k] * dx;
      candidate.State(k + 1) =
          problem_.Dynamics().NextState(candidate.State(k), candidate.Control(k), problem_.TimeStep());
    }

    const double candidate_cost = EvaluateTrajectoryCost(candidate);
    const double actual_decrease = current_cost - candidate_cost;
    const double expected_decrease = -(alpha * expected_linear + alpha * alpha * expected_quadratic);

    // 只有模型预测“应该下降”时，这个 ratio 才有意义。
    if (expected_decrease > 0.0) {
      const double ratio = actual_decrease / expected_decrease;

      // 接受条件包含两层：
      // 1. 真实代价必须下降；
      // 2. 实际下降 / 预测下降 的比值要在合理范围内。
      // 这样可以避免局部模型非常失真时接受一个不可靠步长。
      const bool acceptable_ratio = options_.line_search_accept_lower <= ratio &&
                                    ratio <= options_.line_search_accept_upper;
      if (candidate_cost < current_cost && acceptable_ratio) {
        *accepted_trajectory = candidate;
        *accepted_cost = candidate_cost;
        *accepted_alpha = alpha;
        *accepted_ratio = ratio;
        return true;
      }
    }

    // 当前 α 不可接受，就继续缩小步长再试。
    alpha *= options_.line_search_decrease_factor;
  }

  return false;
}

// Solve 是整个 iLQR 的主流程入口。
// 逻辑顺序如下：
// 1. 检查输入维度；
// 2. 用初始控制序列 rollout 出一条初始轨迹；
// 3. 反复执行：局部展开 -> backward pass -> forward pass；
// 4. 若成本改善足够小则停止；
// 5. 若正则化已经大到上限仍无法推进，则返回当前最好轨迹。
Trajectory ILQRSolver::Solve(const Vector& initial_state, const std::vector<Vector>& initial_controls) {
  if (initial_state.size() != problem_.StateDim()) {
    throw std::invalid_argument("iLQR received an initial state with invalid dimensions.");
  }
  if (static_cast<int>(initial_controls.size()) != problem_.Horizon()) {
    throw std::invalid_argument("iLQR received a control sequence with invalid length.");
  }

  // 清空上一轮求解留下的历史数据。
  cost_history_.clear();
  alpha_history_.clear();
  regularization_history_.clear();
  improvement_ratio_history_.clear();

  // 使用用户给定的控制初值 rollout 出名义轨迹。
  // 注意：这里不是优化结果，只是第一次迭代的线性化中心。
  Trajectory trajectory = problem_.Rollout(initial_state, initial_controls);
  #ifndef NDEBUG
  trajectory.Print();
  #endif
  return trajectory;
  cost_history_.push_back(EvaluateTrajectoryCost(trajectory));

  // 初始正则化至少不能小于下限。
  double regularization = std::max(options_.regularization_init, options_.regularization_min);

  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    const double previous_cost = cost_history_.back();
    bool accepted = false;

    // 一次外层迭代里，如果 backward/forward 失败，会不断调大正则化重试，
    // 直到某个更新被接受，或者正则化超过上限。
    while (!accepted) {
      // 先围绕当前轨迹构建局部模型。
      UpdateExpansions(trajectory);

      double expected_linear = 0.0;
      double expected_quadratic = 0.0;
      const bool backward_ok =
          BackwardPass(trajectory, regularization, &expected_linear, &expected_quadratic);
      if (!backward_ok) {
        regularization *= options_.regularization_increase_factor;
        if (regularization > options_.regularization_max) {
          return trajectory;
        }
        continue;
      }

      Trajectory accepted_trajectory(problem_.StateDim(), problem_.ControlDim(), problem_.Horizon());
      double accepted_cost = previous_cost;
      double accepted_alpha = 0.0;
      double accepted_ratio = 0.0;
      accepted = ForwardPass(initial_state, trajectory, previous_cost, expected_linear,
                             expected_quadratic, &accepted_trajectory, &accepted_cost,
                             &accepted_alpha, &accepted_ratio);
      if (accepted) {
        // 接受更新：替换当前轨迹，并记录调试/分析用的历史数据。
        trajectory = accepted_trajectory;
        cost_history_.push_back(accepted_cost);
        alpha_history_.push_back(accepted_alpha);
        regularization_history_.push_back(regularization);
        improvement_ratio_history_.push_back(accepted_ratio);

        // 当前局部模型表现不错，可以适当减小正则化，让后续步子更“激进”一些。
        regularization =
            std::max(options_.regularization_min,
                     regularization / options_.regularization_decrease_factor);
      } else {
        // 线搜索失败，通常意味着当前二次模型或步长都不理想，增大正则化后重试。
        regularization *= options_.regularization_increase_factor;
        if (regularization > options_.regularization_max) {
          return trajectory;
        }
      }
    }

    // 若最新一次接受更新带来的成本下降已经很小，则认为收敛。
    if (std::abs(previous_cost - cost_history_.back()) < options_.cost_tolerance) {
      break;
    }
  }

  return trajectory;
}

}  // namespace my_al_ilqr
