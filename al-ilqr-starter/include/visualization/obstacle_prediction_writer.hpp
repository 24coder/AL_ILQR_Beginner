#pragma once

#include <filesystem>
#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

// 将动态障碍物预测（圆形近似）导出为 CSV。
//
// 导出数据形状：
// - 每个预测时刻一行。
// - 字段为 k, cx, cy, radius。
// 其中 obstacle_centers[k] 为二维向量 [cx, cy]，radius 为统一半径。
//
// 导出目的：
// - 与车辆轨迹叠加绘制，直观查看避障时空关系；
// - 为测试或复现实验保留外部可读取的障碍物预测序列。
void WriteObstaclePredictionCsv(const std::filesystem::path& path,
                                const std::vector<Vector>& obstacle_centers,
                                double radius);

}  // namespace my_al_ilqr
