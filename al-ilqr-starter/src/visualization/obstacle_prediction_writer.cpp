#include "visualization/obstacle_prediction_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace my_al_ilqr {

void WriteObstaclePredictionCsv(const std::filesystem::path& path,
                                const std::vector<Vector>& obstacle_centers,
                                double radius) {
  // 自动创建输出目录，保证导出流程可直接在测试/脚本中调用。
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  // 打开目标 CSV 文件。
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open obstacle prediction CSV file for writing.");
  }

  // 导出动态障碍物预测轨迹的数据形状：
  // - k: 预测时间步。
  // - cx, cy: 障碍物圆心坐标（世界坐标系）。
  // - radius: 障碍物半径（每个时刻相同，重复写出便于下游无状态解析）。
  out << "k,cx,cy,radius\n";

  // 逐时刻写出障碍物中心预测。每个中心必须是二维向量，否则抛异常。
  for (int k = 0; k < static_cast<int>(obstacle_centers.size()); ++k) {
    if (obstacle_centers[k].size() != 2) {
      throw std::invalid_argument("Obstacle prediction centers must be 2D vectors.");
    }
    out << k << "," << obstacle_centers[k](0) << "," << obstacle_centers[k](1) << "," << radius
        << "\n";
  }
}

}  // namespace my_al_ilqr
