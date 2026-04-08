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

}  // namespace my_al_ilqr
