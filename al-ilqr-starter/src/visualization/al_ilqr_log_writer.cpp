#include "visualization/al_ilqr_log_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace my_al_ilqr {

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

}  // namespace my_al_ilqr
