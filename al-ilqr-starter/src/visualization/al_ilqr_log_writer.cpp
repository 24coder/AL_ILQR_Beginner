#include "visualization/al_ilqr_log_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace my_al_ilqr {

namespace {

std::vector<const ALILQROuterIterationLog*> CollectLogsWithTrajectory(
    const std::vector<ALILQROuterIterationLog>& outer_log) {
  std::vector<const ALILQROuterIterationLog*> valid_logs;
  valid_logs.reserve(outer_log.size());
  for (const auto& log : outer_log) {
    if (log.trajectory.has_value()) {
      valid_logs.push_back(&log);
    }
  }
  return valid_logs;
}

}  // namespace

void WriteALILQROuterLogCsv(const std::filesystem::path& path,
                            const std::vector<ALILQROuterIterationLog>& outer_log) {
  // 若目标路径包含父目录，则先递归创建目录，确保写文件时路径可用。
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  // 打开 CSV 输出文件；失败时抛异常，避免静默丢失日志。
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open AL-iLQR outer log CSV file for writing.");
  }

  // 导出字段说明（供可视化与离线分析使用）：
  // - outer_iteration: 外层 AL 迭代编号。
  // - inner_iterations: 该外层迭代内 iLQR 内层实际执行次数。
  // - base_cost: 原始目标函数值（不含 AL 罚项）。
  // - augmented_cost: 增广拉格朗日目标值（含罚项与对偶项）。
  // - max_violation: 当前轨迹最大约束违反量。
  // - best_violation_so_far: 历史最优（最小）违反量，用于判断收敛趋势。
  // - max_penalty: 当前迭代中的最大罚因子。
  // - penalty_updated: 本次是否触发罚因子更新（1/0）。
  out << "outer_iteration,inner_iterations,base_cost,augmented_cost,max_violation,"
         "best_violation_so_far,max_penalty,penalty_updated\n";

  // 逐条写出外层日志，保持与结构体字段一一对应，便于后处理脚本直接读取。
  for (const auto& log : outer_log) {
    out << log.outer_iteration << "," << log.inner_iterations << "," << log.base_cost << ","
        << log.augmented_cost << "," << log.max_violation << "," << log.best_violation_so_far
        << "," << log.max_penalty << "," << (log.penalty_updated ? 1 : 0) << "\n";
  }
}

void WriteALILQRInnerCostHistoryCsv(const std::filesystem::path& path,
                                    const std::vector<ALILQROuterIterationLog>& outer_log) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open AL-iLQR inner cost history CSV file for writing.");
  }

  // 格式：outer_iter, inner_step, augmented_cost
  // inner_step=0 对应初始 rollout 代价（即第一次 iLQR 迭代前的起点）。
  // 后续每步为接受的 iLQR 优化更新后的增广代价。
  out << "outer_iter,inner_step,augmented_cost\n";

  for (const auto& log : outer_log) {
    for (int step = 0; step < static_cast<int>(log.inner_cost_history.size()); ++step) {
      out << log.outer_iteration << "," << step << "," << log.inner_cost_history[step] << "\n";
    }
  }
}

void WriteALILQRTrajectoryEvolutionCsv(const std::filesystem::path& path,
                                       const std::vector<ALILQROuterIterationLog>& outer_log) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open AL-iLQR trajectory evolution CSV file for writing.");
  }

  // 只取有轨迹快照的日志条目。
  // 格式：k, outer_1_x, outer_1_y, outer_2_x, outer_2_y, ...
  // 每列对应某一外层迭代结束后的轨迹 x/y 坐标。
  const auto valid_logs = CollectLogsWithTrajectory(outer_log);
  if (valid_logs.empty()) {
    return;
  }

  // 写表头：k, outer_1_x, outer_1_y, ...
  out << "k";
  for (const auto* log : valid_logs) {
    out << ",outer_" << log->outer_iteration << "_x"
        << ",outer_" << log->outer_iteration << "_y";
  }
  out << "\n";

  // 以第一个有效轨迹的 horizon 为基准写行
  const int horizon = valid_logs[0]->trajectory->Horizon();
  for (int k = 0; k <= horizon; ++k) {
    out << k;
    for (const auto* log : valid_logs) {
      const auto& traj = *log->trajectory;
      out << "," << traj.State(k)(0) << "," << traj.State(k)(1);
    }
    out << "\n";
  }
}

void WriteALILQRTrajectoryEvolutionMetaCsv(
    const std::filesystem::path& path,
    const std::vector<ALILQROuterIterationLog>& outer_log) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error(
        "Failed to open AL-iLQR trajectory evolution meta CSV file for writing.");
  }

  // 仅导出带轨迹快照的外层迭代，使其与 trajectory evolution CSV 中的帧一一对应。
  out << "outer_iteration,inner_iterations,base_cost,augmented_cost,max_violation,"
         "best_violation_so_far,max_penalty,penalty_updated\n";

  const auto valid_logs = CollectLogsWithTrajectory(outer_log);
  for (const auto* log : valid_logs) {
    out << log->outer_iteration << "," << log->inner_iterations << "," << log->base_cost << ","
        << log->augmented_cost << "," << log->max_violation << ","
        << log->best_violation_so_far << "," << log->max_penalty << ","
        << (log->penalty_updated ? 1 : 0) << "\n";
  }
}

}  // namespace my_al_ilqr
