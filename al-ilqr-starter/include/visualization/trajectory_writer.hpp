#pragma once

#include <filesystem>

#include "core/trajectory.hpp"

namespace my_al_ilqr {

// 将轨迹导出为 CSV 文件，供可视化与离线分析使用。
//
// 导出数据形状：
// - 行数：Horizon + 1（每个状态结点一行）。
// - 列：k, x0..x(n-1), u0..u(m-1)。
// - 最后一行没有实际控制量，控制列保持为空占位，保证列数恒定。
void WriteTrajectoryCsv(const std::filesystem::path& path, const Trajectory& trajectory);

}  // namespace my_al_ilqr
