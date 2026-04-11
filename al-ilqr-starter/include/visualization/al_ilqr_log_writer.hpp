#pragma once

#include <filesystem>
#include <vector>

#include "al/al_ilqr_solver.hpp"

namespace my_al_ilqr {

// 将 AL-iLQR 外层迭代日志导出为 CSV。
//
// 导出目的：
// 1) 支持训练/调参时观察约束违反量与罚因子演化；
// 2) 支持可视化收敛曲线（cost、violation、penalty）；
// 3) 支持回归测试中对外层行为进行离线比对。
//
// 数据形状：每个 ALILQROuterIterationLog 对应 CSV 一行，字段顺序与实现中的表头一致。
void WriteALILQROuterLogCsv(const std::filesystem::path& path,
                            const std::vector<ALILQROuterIterationLog>& outer_log);

// 将每个外层 AL 迭代内部的 iLQR 代价历史导出为 CSV。
//
// 格式：outer_iter, inner_step, augmented_cost
// 每条记录对应某一外层迭代中，内层 iLQR 某一被接受的迭代步的增广代价值。
// inner_step=0 对应初始 rollout 代价，后续每步为接受的优化更新后的代价。
//
// 用途：直观验证内层 iLQR 每轮是否真正在减小增广目标函数。
void WriteALILQRInnerCostHistoryCsv(const std::filesystem::path& path,
                                    const std::vector<ALILQROuterIterationLog>& outer_log);

// 将每个外层 AL 迭代后的完整轨迹快照导出为 CSV。
//
// 格式：k, outer_1_x, outer_1_y, outer_2_x, outer_2_y, ...
// 每列对应某一外层迭代结束后的轨迹 x/y 坐标，每行为轨迹的一个时间步。
//
// 用途：可视化轨迹随 AL 迭代的演化过程，直观看到"绕障轨迹逐步成形"的过程。
void WriteALILQRTrajectoryEvolutionCsv(const std::filesystem::path& path,
                                       const std::vector<ALILQROuterIterationLog>& outer_log);

// 将与轨迹快照一一对应的外层迭代元数据导出为 CSV。
//
// 格式：outer_iteration, inner_iterations, base_cost, augmented_cost,
//      max_violation, best_violation_so_far, max_penalty, penalty_updated
//
// 仅导出带 trajectory 快照的外层迭代，保证与 trajectory evolution CSV 中的每一帧严格对齐。
// 用途：供 matplotlib / pandas 等脚本直接给每帧动画添加标题、说明文字与收敛指标。
void WriteALILQRTrajectoryEvolutionMetaCsv(
    const std::filesystem::path& path,
    const std::vector<ALILQROuterIterationLog>& outer_log);

}  // namespace my_al_ilqr
