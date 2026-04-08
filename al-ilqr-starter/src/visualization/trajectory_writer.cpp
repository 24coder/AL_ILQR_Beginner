#include "visualization/trajectory_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace my_al_ilqr {

void WriteTrajectoryCsv(const std::filesystem::path& path, const Trajectory& trajectory) {
  // 若用户给定了父目录，先创建目录层级，避免输出失败。
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  // 打开 CSV 文件，若失败则抛错，确保调用方能及时发现导出问题。
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open trajectory CSV file for writing.");
  }

  // CSV 列定义：
  // - k: 离散时间步。
  // - x0..x(n-1): 状态向量各维。
  // - u0..u(m-1): 控制向量各维。
  // 该数据形状用于轨迹可视化、控制曲线绘制和回归测试比对。
  out << "k";
  for (int i = 0; i < trajectory.StateDim(); ++i) {
    out << ",x" << i;
  }
  for (int i = 0; i < trajectory.ControlDim(); ++i) {
    out << ",u" << i;
  }
  out << "\n";

  // 逐时刻写数据：状态有 N+1 个结点，控制有 N 个结点。
  // 对最后一个状态结点（k == Horizon）没有对应控制，因此输出空列占位，
  // 以保持每行列数一致，方便 pandas/Excel 等工具直接读取。
  for (int k = 0; k <= trajectory.Horizon(); ++k) {
    out << k;
    for (int i = 0; i < trajectory.StateDim(); ++i) {
      out << "," << trajectory.State(k)(i);
    }
    if (k < trajectory.Horizon()) {
      for (int i = 0; i < trajectory.ControlDim(); ++i) {
        out << "," << trajectory.Control(k)(i);
      }
    } else {
      for (int i = 0; i < trajectory.ControlDim(); ++i) {
        out << ",";
      }
    }
    out << "\n";
  }
}

}  // namespace my_al_ilqr
