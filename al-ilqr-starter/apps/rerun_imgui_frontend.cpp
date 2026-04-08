#include <algorithm>    // 提供 std::min/std::max/std::clamp 等通用算法。
#include <array>        // 提供定长数组 std::array，用于配置路径缓冲区。
#include <chrono>       // 提供计时工具，用于统计规划耗时。
#include <cmath>        // 提供三角函数/绝对值/有限值判断等数学工具。
#include <cstdio>       // 提供 std::snprintf，用于格式化文本。
#include <filesystem>   // 提供路径与目录操作，用于 CSV 路径与 build 目录创建。
#include <future>       // 提供 std::future/std::async，用于后台异步规划。
#include <limits>       // 提供数值上下界与 NaN/Inf 常量。
#include <sstream>      // 提供字符串流，拼装状态文本与终端状态。
#include <stdexcept>    // 提供标准异常类型。
#include <string>       // 提供 std::string。
#include <utility>      // 提供 std::pair。
#include <vector>       // 提供动态数组，承载轨迹序列与历史曲线。

#include <GLFW/glfw3.h>                    // GLFW：窗口、输入、OpenGL 上下文管理。
#include <imgui.h>                         // Dear ImGui 核心 API。
#include <backends/imgui_impl_glfw.h>      // ImGui GLFW 平台后端。
#include <backends/imgui_impl_opengl3.h>   // ImGui OpenGL3 渲染后端。

#include "al/al_ilqr_solver.hpp"                 // AL-iLQR 求解器与迭代日志定义。
#include "autodrive/demo_scenario.hpp"           // 测试场景构造（单静态障碍）。
#include "autodrive/vehicle_bicycle_config.hpp"  // 车辆模型配置加载与默认配置路径。
#include "visualization/trajectory_writer.hpp"   // 轨迹写 CSV 的工具函数。

namespace {  // 匿名命名空间：本文件私有实现细节。

// 本前端 Demo 的定位：
// 1) 通过 ImGui 提供“参数可视化调参 + 一键规划”；
// 2) 将 AL-iLQR 规划结果（轨迹、约束违反、罚参数演化）在同一窗口内实时展示；
// 3) 保留 CSV 导出能力，方便离线分析与复现实验。
//
// 核心执行流：
// - 左侧 Controls：加载配置、修改车辆/场景参数、触发 Plan。
// - 中间 Visualization：显示参考线、障碍、初始猜测、优化轨迹及车辆包络。
// - 右下 Debug：显示速度/曲率/加速度及 outer loop 指标曲线。
// - Plan 按钮触发后台异步求解，主线程持续刷新 UI，求解完成后原子替换结果。
using my_al_ilqr::ALILQRSolver;              // 引入求解器类型别名，减少命名空间前缀噪声。
using my_al_ilqr::ALILQROuterIterationLog;   // 引入外层迭代日志结构类型。
using my_al_ilqr::StaticObstacleScenarioConfig;  // 引入静态障碍场景参数结构。
using my_al_ilqr::VehicleBicycleConfig;      // 引入车辆自行车模型参数结构。
using my_al_ilqr::Vector;                    // 引入线性代数向量类型（当前文件未直接使用，保持接口一致性）。

constexpr double kPi = 3.14159265358979323846;  // π 常量，避免多处硬编码。
constexpr double kTwoPi = 2.0 * kPi;            // 2π 常量，圆周采样时更直观。

// 后台规划任务输出：聚合了求解状态、时序统计、轨迹序列和可视化曲线数据。
// 设计目标是让 UI 线程在一次状态更新中拿到“完整快照”，避免多处拼装导致状态不一致。
struct PlanningOutput {
  bool success = false;                      // 规划任务是否成功完成（无异常且有有效结果）。
  std::string status;                        // 面向 UI 状态栏的综合文本信息。
  std::string terminal_state;                // 终点状态向量文本（便于快速核对终态）。
  bool converged = false;                    // 求解器收敛标志。
  double best_violation = 0.0;               // 全过程最优约束违反值。
  double initial_violation = 0.0;            // 初始轨迹最大违反值。
  double final_violation = 0.0;              // 结果轨迹最大违反值。
  double initial_cost = 0.0;                 // 初始轨迹总代价。
  double final_cost = 0.0;                   // 优化后轨迹总代价。
  double planning_time_ms = 0.0;             // 规划总耗时（毫秒）。
  int horizon = 0;                           // 轨迹时域长度 N。
  int outer_iterations = 0;                  // AL 外层迭代次数。
  int total_inner_iterations = 0;            // 所有外层累计内层迭代次数。
  std::filesystem::path trajectory_csv_path; // 导出的轨迹 CSV 文件路径。
  VehicleBicycleConfig vehicle_config;       // 当次规划使用的车辆参数快照。
  StaticObstacleScenarioConfig scenario_config;  // 当次规划使用的场景参数快照。
  std::vector<ALILQROuterIterationLog> outer_logs;  // 外层迭代日志列表。
  std::vector<double> initial_x;             // 初始猜测轨迹 x 序列。
  std::vector<double> initial_y;             // 初始猜测轨迹 y 序列。
  std::vector<double> x;                     // 优化后轨迹 x 序列。
  std::vector<double> y;                     // 优化后轨迹 y 序列。
  std::vector<double> yaw;                   // 优化后轨迹航向角序列。
  std::vector<double> speed;                 // 优化后轨迹速度序列。
  std::vector<double> accel;                 // 优化后控制量加速度序列（长度 N）。
  std::vector<double> kappa;                 // 优化后曲率序列（由转角与轴距换算）。
  std::vector<double> base_cost_history;     // 外层迭代基础代价历史曲线。
  std::vector<double> violation_history;     // 外层迭代最大违反历史曲线。
  std::vector<double> penalty_history;       // 外层迭代最大罚参数历史曲线。
};

// 仅用于“参数预览”的轻量场景输出，避免每次调参都触发完整优化。
struct ScenePreview {
  bool valid = false;                        // 预览是否生成成功。
  std::string status;                        // 预览构建状态（成功/异常信息）。
  VehicleBicycleConfig vehicle_config;       // 预览对应车辆参数。
  StaticObstacleScenarioConfig scenario_config;  // 预览对应场景参数。
  std::vector<double> x;                     // 初始 rollout 的 x。
  std::vector<double> y;                     // 初始 rollout 的 y。
  std::vector<double> yaw;                   // 初始 rollout 的 yaw。
  std::vector<double> speed;                 // 初始 rollout 的速度。
};

// 前端全局状态：
// - 参数区（config_path, vehicle_config, scenario_config）
// - 可视化开关区（show_*）
// - 运行状态区（planning, has_result, preview_dirty）
// - 视角区（zoom/pan）
// - 数据区（preview/result）
struct FrontendState {
  std::array<char, 512> config_path{};       // 配置路径输入缓冲（固定容量便于 ImGui InputText）。
  VehicleBicycleConfig vehicle_config;       // 当前可编辑车辆参数。
  StaticObstacleScenarioConfig scenario_config;  // 当前可编辑场景参数。
  std::string status = "Load a config and press Plan.";  // 状态文本。
  std::string terminal_state;                // 最近一次规划终态文本。
  std::filesystem::path last_csv_path;       // 最近一次导出的 CSV 路径。
  double last_best_violation = 0.0;          // 最近一次最优违反值。
  bool planning = false;                     // 是否正在后台规划。
  bool has_result = false;                   // 是否持有有效规划结果。
  bool preview_dirty = true;                 // 预览是否需要按最新参数重建。
  double scene_zoom = 1.0;                   // 场景视图缩放因子。
  double scene_pan_x = 0.0;                  // 场景视图平移 x。
  double scene_pan_y = 0.0;                  // 场景视图平移 y。
  bool show_reference_line = true;           // 是否显示参考线。
  bool show_obstacle = true;                 // 是否显示障碍物。
  bool show_initial_guess = true;            // 是否显示初始猜测轨迹。
  bool show_optimized_trajectory = true;     // 是否显示优化轨迹。
  bool show_vehicle_outline = true;          // 是否显示车辆矩形轮廓。
  bool show_safety_circle = true;            // 是否显示车辆安全圆近似。
  ScenePreview preview;                      // 当前预览数据。
  PlanningOutput result;                     // 当前规划结果数据。
};

// 二维包围盒，用于世界坐标 -> 画布像素坐标映射。
struct Bounds2D {
  double min_x = 0.0;                        // x 最小值。
  double max_x = 1.0;                        // x 最大值。
  double min_y = 0.0;                        // y 最小值。
  double max_y = 1.0;                        // y 最大值。
};

// 统一的“标签 + 滑条 + 数值输入框”控件：
// 便于在参数面板中快速微调并保证布局一致。
bool SliderInputDouble(const char* label,
                       double* value,
                       double min_value,
                       double max_value,
                       const char* format = "%.3f") {
  bool changed = false;                              // 记录本控件是否发生值变化。
  ImGui::PushID(label);                              // 用 label 作为 ID 域，避免子控件 ID 冲突。
  const float full_width = ImGui::GetContentRegionAvail().x;  // 当前可用总宽度。
  const float label_width = 120.0f;                  // 标签区固定宽。
  const float input_width = 88.0f;                   // 数值输入框固定宽。
  const float spacing = ImGui::GetStyle().ItemSpacing.x;  // 组件间默认水平间距。
  const float slider_width =
      std::max(80.0f, full_width - label_width - input_width - 2.0f * spacing);  // 剩余宽给滑条，且保证下限。
  ImGui::AlignTextToFramePadding();                  // 标签与控件基线对齐。
  ImGui::TextUnformatted(label);                     // 绘制标签文本（不做格式化解析）。
  ImGui::SameLine(label_width);                      // 同行切换到指定 x 偏移。
  ImGui::SetNextItemWidth(slider_width);             // 设定下一控件（滑条）宽度。
  changed |= ImGui::SliderScalar("##slider", ImGuiDataType_Double, value, &min_value, &max_value, format);  // 双精度滑条。
  ImGui::SameLine(0.0f, spacing);                    // 滑条后同一行接输入框。
  ImGui::SetNextItemWidth(input_width);              // 设定输入框宽度。
  changed |= ImGui::InputScalar("##input", ImGuiDataType_Double, value, nullptr, nullptr, format);  // 双精度输入框。
  ImGui::PopID();                                    // 结束 ID 域。
  return changed;                                    // 返回是否有改动。
}

// 将世界坐标 (x,y) 映射到画布像素坐标。
ImVec2 ToCanvas(const ImVec2& origin, const ImVec2& size, const Bounds2D& bounds, double x, double y) {
  const double span_x = std::max(1e-6, bounds.max_x - bounds.min_x);  // x 轴跨度，防除零。
  const double span_y = std::max(1e-6, bounds.max_y - bounds.min_y);  // y 轴跨度，防除零。
  const float px = origin.x + static_cast<float>((x - bounds.min_x) / span_x) * size.x;  // 线性映射到像素 x。
  const float py = origin.y + size.y - static_cast<float>((y - bounds.min_y) / span_y) * size.y;  // y 方向翻转（屏幕坐标向下增）。
  return ImVec2(px, py);  // 返回映射点。
}

// 计算场景边界：综合 preview、优化轨迹、初始轨迹、障碍圆，
// 再附加 margin，确保可视化时关键元素不贴边。
Bounds2D ComputeSceneBounds(const ScenePreview& preview, const PlanningOutput* output) {
  Bounds2D bounds;                                         // 初始化边界容器。
  bounds.min_x = std::numeric_limits<double>::infinity();  // 先置 +Inf，后续逐点收缩。
  bounds.max_x = -std::numeric_limits<double>::infinity(); // 先置 -Inf，后续逐点扩展。
  bounds.min_y = std::numeric_limits<double>::infinity();  // 同上。
  bounds.max_y = -std::numeric_limits<double>::infinity(); // 同上。

  auto expand = [&](double x, double y) {                 // 小工具：将一点纳入边界。
    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_y = std::min(bounds.min_y, y);
    bounds.max_y = std::max(bounds.max_y, y);
  };

  for (size_t i = 0; i < preview.x.size(); ++i) {         // 纳入 preview 轨迹。
    expand(preview.x[i], preview.y[i]);
  }
  if (output) {                                            // 若有优化结果，则额外纳入结果轨迹与初始猜测轨迹。
    for (size_t i = 0; i < output->x.size(); ++i) {
      expand(output->x[i], output->y[i]);
    }
    for (size_t i = 0; i < output->initial_x.size(); ++i) {
      expand(output->initial_x[i], output->initial_y[i]);
    }
  }
  expand(preview.scenario_config.obstacle_center_x - preview.scenario_config.obstacle_radius,
         preview.scenario_config.obstacle_center_y - preview.scenario_config.obstacle_radius);  // 纳入障碍圆左下。
  expand(preview.scenario_config.obstacle_center_x + preview.scenario_config.obstacle_radius,
         preview.scenario_config.obstacle_center_y + preview.scenario_config.obstacle_radius);  // 纳入障碍圆右上。

  if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.max_x)) {  // 若无有效点，回退默认边界。
    bounds = Bounds2D{};
  }

  const double margin_x = std::max(1.0, 0.08 * (bounds.max_x - bounds.min_x));  // x 方向边距。
  const double margin_y = std::max(0.8, 0.15 * (bounds.max_y - bounds.min_y));  // y 方向边距。
  bounds.min_x -= margin_x;                           // 左扩。
  bounds.max_x += margin_x;                           // 右扩。
  bounds.min_y -= margin_y;                           // 下扩。
  bounds.max_y += margin_y;                           // 上扩。
  return bounds;                                      // 返回最终边界。
}

// 在原始边界基础上应用缩放与平移视角。
Bounds2D ApplySceneView(const Bounds2D& input_bounds, double zoom, double pan_x, double pan_y) {
  Bounds2D bounds = input_bounds;                     // 复制输入边界，输出可修改版本。
  zoom = std::clamp(zoom, 0.25, 20.0);                // 限制缩放范围，避免极端数值。
  const double cx = 0.5 * (bounds.min_x + bounds.max_x) + pan_x;  // 视图中心 x（含平移）。
  const double cy = 0.5 * (bounds.min_y + bounds.max_y) + pan_y;  // 视图中心 y（含平移）。
  const double span_x = std::max(1e-6, (bounds.max_x - bounds.min_x) / zoom);  // 缩放后跨度 x。
  const double span_y = std::max(1e-6, (bounds.max_y - bounds.min_y) / zoom);  // 缩放后跨度 y。
  bounds.min_x = cx - 0.5 * span_x;                   // 由中心与跨度反推边界。
  bounds.max_x = cx + 0.5 * span_x;
  bounds.min_y = cy - 0.5 * span_y;
  bounds.max_y = cy + 0.5 * span_y;
  return bounds;                                      // 返回视图变换后的边界。
}

// 生成圆形折线（用于障碍物与安全圆显示）。
std::vector<ImVec2> MakeObstaclePolyline(const ImVec2& origin,
                                         const ImVec2& size,
                                         const Bounds2D& bounds,
                                         double cx,
                                         double cy,
                                         double radius,
                                         int segments = 48) {
  std::vector<ImVec2> pts;                            // 输出点序列。
  pts.reserve(segments + 1);                          // 闭环需回到起点，所以是 +1。
  for (int i = 0; i <= segments; ++i) {               // 逐段角度采样。
    const double theta = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);  // 当前采样角。
    pts.push_back(ToCanvas(origin, size, bounds,
                           cx + radius * std::cos(theta),
                           cy + radius * std::sin(theta)));  // 世界圆点 -> 画布坐标。
  }
  return pts;                                         // 返回闭合折线点序列。
}

// 生成车辆矩形轮廓折线（世界坐标先旋转再平移，然后映射到画布）。
std::vector<ImVec2> MakeVehicleOutline(const VehicleBicycleConfig& config,
                                       const ImVec2& origin,
                                       const ImVec2& size,
                                       const Bounds2D& bounds,
                                       double x,
                                       double y,
                                       double yaw) {
  const double rear = config.body.rear_axle_to_rear;                 // 后轴到车尾距离。
  const double front = config.body.length - config.body.rear_axle_to_rear;  // 后轴到车头距离。
  const double half_width = 0.5 * config.body.width;                 // 半车宽。
  const double c = std::cos(yaw);                                    // 朝向余弦。
  const double s = std::sin(yaw);                                    // 朝向正弦。

  const std::vector<std::pair<double, double>> body_corners = {      // 车体局部坐标角点，最后一点重复首点形成闭环。
      {front, half_width},
      {front, -half_width},
      {-rear, -half_width},
      {-rear, half_width},
      {front, half_width},
  };

  std::vector<ImVec2> polyline;                                      // 输出画布折线。
  polyline.reserve(body_corners.size());                             // 预分配。
  for (const auto& corner : body_corners) {                          // 将每个局部角点变换到世界坐标。
    const double wx = x + c * corner.first - s * corner.second;      // 2D 刚体变换：旋转 + 平移。
    const double wy = y + s * corner.first + c * corner.second;
    polyline.push_back(ToCanvas(origin, size, bounds, wx, wy));      // 世界 -> 画布。
  }
  return polyline;                                                    // 返回车辆轮廓折线。
}

// 绘制实线折线（通用）。
void DrawPolyline(ImDrawList* draw_list, const std::vector<ImVec2>& polyline, ImU32 color, float thickness) {
  if (!draw_list || polyline.size() < 2) {                          // 至少两点才能连线。
    return;
  }
  draw_list->AddPolyline(polyline.data(), static_cast<int>(polyline.size()), color, 0, thickness);  // 一次性提交折线。
}

// 填充闭合凸多边形（用于车辆矩形半透明填充）。
void FillClosedPolygon(ImDrawList* draw_list, const std::vector<ImVec2>& polygon, ImU32 fill_color) {
  if (!draw_list || polygon.size() < 3) {                           // 至少三点才是多边形。
    return;
  }
  const int count = static_cast<int>(polygon.size()) - 1;           // 最后一点通常与首点重复，填充时去掉尾点。
  if (count >= 3) {                                                  // 再次确保有效顶点数。
    draw_list->AddConvexPolyFilled(polygon.data(), count, fill_color);  // 绘制凸多边形填充。
  }
}

// 绘制虚线折线：按 stride 跳段绘制。
void DrawDashedPolyline(ImDrawList* draw_list,
                        const std::vector<ImVec2>& polyline,
                        ImU32 color,
                        float thickness,
                        int draw_stride = 2) {
  if (!draw_list || polyline.size() < 2) {                          // 输入有效性检查。
    return;
  }
  for (size_t i = 0; i + 1 < polyline.size(); ++i) {                // 逐线段处理。
    if ((static_cast<int>(i) % draw_stride) != 0) {                 // 非目标段跳过，形成虚线效果。
      continue;
    }
    draw_list->AddLine(polyline[i], polyline[i + 1], color, thickness);  // 绘制保留段。
  }
}

// 绘制单条信号曲线面板（带坐标轴、0 线、min/max 标注）。
void DrawSignalPlot(const char* title,
                    const std::vector<double>& values,
                    ImU32 color,
                    double fixed_min = std::numeric_limits<double>::quiet_NaN(),
                    double fixed_max = std::numeric_limits<double>::quiet_NaN()) {
  ImGui::BeginChild(title, ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar);  // 子窗口承载单图。
  ImGui::TextUnformatted(title);                                                         // 图标题。
  const ImVec2 top_left = ImGui::GetCursorScreenPos();                                  // 画布左上角。
  const ImVec2 canvas_size = ImVec2(std::max(80.0f, ImGui::GetContentRegionAvail().x),
                                    std::max(100.0f, ImGui::GetContentRegionAvail().y - 8.0f));  // 画布尺寸，设最小值。
  const ImVec2 bottom_right(top_left.x + canvas_size.x, top_left.y + canvas_size.y);    // 画布右下角。
  ImDrawList* draw_list = ImGui::GetWindowDrawList();                                    // 当前窗口绘制列表。
  draw_list->AddRectFilled(top_left, bottom_right, IM_COL32(19, 23, 31, 255), 6.0f);    // 背景填充。
  draw_list->AddRect(top_left, bottom_right, IM_COL32(70, 78, 96, 255), 6.0f, 0, 1.0f); // 外框。

  if (values.size() >= 2) {                                            // 至少两个点才画曲线。
    double min_value = std::numeric_limits<double>::infinity();        // 动态最小值初始化。
    double max_value = -std::numeric_limits<double>::infinity();       // 动态最大值初始化。
    for (double value : values) {                                       // 扫描有效值范围。
      if (!std::isfinite(value)) {                                      // 忽略 NaN/Inf。
        continue;
      }
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
    if (std::isfinite(fixed_min)) {                                     // 若指定固定最小值，覆盖动态值。
      min_value = fixed_min;
    }
    if (std::isfinite(fixed_max)) {                                     // 若指定固定最大值，覆盖动态值。
      max_value = fixed_max;
    }
    if (!std::isfinite(min_value) || !std::isfinite(max_value)) {       // 极端情况下回退默认范围。
      min_value = -1.0;
      max_value = 1.0;
    }
    if (std::abs(max_value - min_value) < 1e-6) {                       // 避免零幅度导致归一化除零。
      min_value -= 1.0;
      max_value += 1.0;
    }

    const float pad_x = 10.0f;                                          // 图内左右内边距。
    const float pad_y = 10.0f;                                          // 图内上下内边距。
    const float plot_w = canvas_size.x - 2.0f * pad_x;                  // 可绘图区宽。
    const float plot_h = canvas_size.y - 2.0f * pad_y;                  // 可绘图区高。
    const float x_axis_y = bottom_right.y - pad_y;                      // x 轴像素 y。
    const float y_axis_x = top_left.x + pad_x;                          // y 轴像素 x。
    draw_list->AddLine(ImVec2(y_axis_x, top_left.y + pad_y),
                       ImVec2(y_axis_x, x_axis_y),
                       IM_COL32(110, 118, 138, 255), 1.0f);             // 绘制 y 轴。
    draw_list->AddLine(ImVec2(y_axis_x, x_axis_y),
                       ImVec2(bottom_right.x - pad_x, x_axis_y),
                       IM_COL32(110, 118, 138, 255), 1.0f);             // 绘制 x 轴。
    const float zero_t = static_cast<float>((0.0 - min_value) / (max_value - min_value));  // 0 值在竖直方向归一化位置。
    if (zero_t >= 0.0f && zero_t <= 1.0f) {                              // 0 在可视范围内时绘制零线。
      const float zero_y = bottom_right.y - pad_y - zero_t * plot_h;
      draw_list->AddLine(ImVec2(top_left.x + pad_x, zero_y),
                         ImVec2(bottom_right.x - pad_x, zero_y),
                         IM_COL32(80, 88, 108, 255), 1.0f);             // 绘制零水平线。
      draw_list->AddText(ImVec2(bottom_right.x - pad_x - 28.0f, zero_y - 14.0f),
                         IM_COL32(150, 158, 176, 255), "0");          // 零线文字标记。
    }

    std::vector<ImVec2> points;                                          // 曲线像素点。
    points.reserve(values.size());                                       // 预分配提高效率。
    for (size_t i = 0; i < values.size(); ++i) {
      const float tx = static_cast<float>(i) / static_cast<float>(values.size() - 1);       // 横轴按索引均匀归一化。
      const float ty = static_cast<float>((values[i] - min_value) / (max_value - min_value)); // 纵轴按值归一化。
      points.emplace_back(top_left.x + pad_x + tx * plot_w,
                          bottom_right.y - pad_y - ty * plot_h);         // 归一化 -> 像素坐标。
    }
    DrawPolyline(draw_list, points, color, 2.0f);                        // 绘制曲线主线。

    char label[64];                                                       // 文本缓存。
    std::snprintf(label, sizeof(label), "max %.3f", max_value);         // 最大值文本。
    draw_list->AddText(ImVec2(top_left.x + pad_x, top_left.y + 4.0f), IM_COL32(190, 198, 215, 255), label);  // 写 max。
    std::snprintf(label, sizeof(label), "min %.3f", min_value);         // 最小值文本。
    draw_list->AddText(ImVec2(top_left.x + pad_x, bottom_right.y - 18.0f), IM_COL32(190, 198, 215, 255), label);  // 写 min。
    draw_list->AddText(ImVec2(bottom_right.x - 58.0f, bottom_right.y - 18.0f),
                       IM_COL32(190, 198, 215, 255), "step");          // 横轴说明（step 索引）。
  } else {
    draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f), IM_COL32(180, 180, 180, 255), "No data");  // 无数据提示。
  }

  ImGui::Dummy(canvas_size);                                              // 占位，确保 child 高度正确。
  ImGui::EndChild();                                                      // 结束子窗口。
}

// 绘制图例条目（可选虚线样式，且根据 enabled 变灰）。
void DrawLegendEntry(ImDrawList* draw_list,
                     const ImVec2& anchor,
                     ImU32 color,
                     const char* label,
                     bool enabled,
                     bool dashed = false) {
  const ImVec2 p0(anchor.x, anchor.y + 6.0f);                             // 图例线段起点。
  const ImVec2 p1(anchor.x + 18.0f, anchor.y + 6.0f);                     // 图例线段终点。
  if (enabled) {                                                           // 启用状态：按指定颜色绘制。
    if (dashed) {                                                          // 虚线：拆成两段中间留空。
      draw_list->AddLine(p0, ImVec2(anchor.x + 8.0f, anchor.y + 6.0f), color, 2.0f);
      draw_list->AddLine(ImVec2(anchor.x + 10.0f, anchor.y + 6.0f), p1, color, 2.0f);
    } else {                                                               // 实线：一段直连。
      draw_list->AddLine(p0, p1, color, 2.0f);
    }
  } else {                                                                 // 关闭状态：灰色细线弱化。
    draw_list->AddLine(p0, p1, IM_COL32(90, 90, 90, 120), 1.0f);
  }
  draw_list->AddText(ImVec2(anchor.x + 24.0f, anchor.y), enabled ? IM_COL32(230, 230, 230, 255)
                                                                 : IM_COL32(120, 120, 120, 180),
                     label);                                               // 右侧图例文字。
}

// 主场景绘制：
// 1) 背景与坐标映射
// 2) 参考线/障碍/初始轨迹/优化轨迹
// 3) 车辆矩形与安全圆采样
// 4) 图例与交互提示（滚轮缩放、右键平移）
void DrawSceneView(FrontendState* state) {
  if (!state || !state->preview.valid) {                                  // 无状态或预览无效则不绘制。
    return;
  }
  const ScenePreview& preview = state->preview;                           // 预览引用，简化后续访问。
  const PlanningOutput* output = state->has_result ? &state->result : nullptr;  // 有结果时指向 result，否则为空。
  const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();                  // 场景面板左上屏幕坐标。
  const ImVec2 avail = ImGui::GetContentRegionAvail();                    // 可用绘图区域尺寸。
  const ImVec2 canvas_size(std::max(200.0f, avail.x), std::max(220.0f, avail.y));  // 限制最小尺寸。
  const ImVec2 canvas_max(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);  // 面板右下。
  ImDrawList* draw_list = ImGui::GetWindowDrawList();                     // 获取绘图列表。

  draw_list->AddRectFilled(canvas_pos, canvas_max, IM_COL32(15, 18, 24, 255), 8.0f);     // 背景。
  draw_list->AddRect(canvas_pos, canvas_max, IM_COL32(65, 72, 88, 255), 8.0f, 0, 1.0f);   // 边框。

  const float pad = 18.0f;                                                // 内边距。
  const ImVec2 plot_origin(canvas_pos.x + pad, canvas_pos.y + pad);       // 绘图区左上。
  const ImVec2 plot_size(canvas_size.x - 2.0f * pad, canvas_size.y - 2.0f * pad);  // 绘图区大小。
  const bool hovered = ImGui::IsMouseHoveringRect(canvas_pos, canvas_max, true);    // 鼠标是否悬停在画布。
  if (hovered) {                                                           // 仅在悬停时响应缩放/平移。
    ImGuiIO& io = ImGui::GetIO();                                          // 输入状态。
    if (std::abs(io.MouseWheel) > 1e-6f) {                                 // 滚轮有输入时调整缩放。
      const double zoom_factor = io.MouseWheel > 0.0f ? 1.12 : 1.0 / 1.12; // 向上放大、向下缩小。
      state->scene_zoom = std::clamp(state->scene_zoom * zoom_factor, 0.25, 20.0);  // 应用并限幅。
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {                  // 右键拖拽平移视图。
      const ImVec2 delta = ImGui::GetIO().MouseDelta;                      // 本帧鼠标像素位移。
      const Bounds2D raw_bounds = ComputeSceneBounds(preview, output);     // 当前数据原始边界。
      const Bounds2D viewed_bounds =
          ApplySceneView(raw_bounds, state->scene_zoom, state->scene_pan_x, state->scene_pan_y);  // 应用当前视图参数。
      const double span_x = viewed_bounds.max_x - viewed_bounds.min_x;     // 当前可见世界宽。
      const double span_y = viewed_bounds.max_y - viewed_bounds.min_y;     // 当前可见世界高。
      state->scene_pan_x -= static_cast<double>(delta.x) * span_x / std::max(1.0f, plot_size.x);  // 像素拖拽换算到世界平移。
      state->scene_pan_y += static_cast<double>(delta.y) * span_y / std::max(1.0f, plot_size.y);  // y 方向符号相反。
    }
  }
  const Bounds2D bounds =
      ApplySceneView(ComputeSceneBounds(preview, output), state->scene_zoom, state->scene_pan_x, state->scene_pan_y);  // 最终显示边界。

  const auto ref_a = ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, 0.0);  // 参考线左端（y=0）。
  const auto ref_b = ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, 0.0);  // 参考线右端（y=0）。
  if (state->show_reference_line) {                                        // 根据开关绘制参考线。
    draw_list->AddLine(ref_a, ref_b, IM_COL32(242, 198, 65, 220), 2.0f);
    draw_list->AddText(ImVec2(ref_a.x + 8.0f, ref_a.y - 18.0f), IM_COL32(242, 198, 65, 255), "reference line");
  }

  if (state->show_obstacle) {                                              // 根据开关绘制障碍圆。
    const auto obstacle = MakeObstaclePolyline(plot_origin,
                                               plot_size,
                                               bounds,
                                               preview.scenario_config.obstacle_center_x,
                                               preview.scenario_config.obstacle_center_y,
                                               preview.scenario_config.obstacle_radius);
    DrawPolyline(draw_list, obstacle, IM_COL32(230, 84, 84, 255), 2.5f);
  }

  if (state->show_initial_guess && preview.x.size() >= 2) {                // 绘制初始猜测轨迹（虚线）。
    std::vector<ImVec2> initial_polyline;
    initial_polyline.reserve(preview.x.size());
    for (size_t i = 0; i < preview.x.size(); ++i) {
      initial_polyline.push_back(ToCanvas(plot_origin, plot_size, bounds, preview.x[i], preview.y[i]));
    }
    DrawDashedPolyline(draw_list, initial_polyline, IM_COL32(160, 160, 160, 220), 2.0f, 2);
  }

  if (output && state->show_optimized_trajectory && output->x.size() >= 2) {  // 有结果且开关打开时绘制优化轨迹。
    std::vector<ImVec2> trajectory_polyline;
    trajectory_polyline.reserve(output->x.size());
    for (size_t i = 0; i < output->x.size(); ++i) {
      trajectory_polyline.push_back(ToCanvas(plot_origin, plot_size, bounds, output->x[i], output->y[i]));
    }
    DrawPolyline(draw_list, trajectory_polyline, IM_COL32(72, 149, 239, 255), 3.6f);  // 主轨迹加粗显示。

    const auto vehicle_circle =
        my_al_ilqr::BuildSingleCircleVehicleApproximation(output->vehicle_config.body);  // 读取单圆车辆近似参数。
    const int sample_step = std::max(1, static_cast<int>(output->x.size() / 8));          // 轨迹稀疏采样，避免过密叠画。
    for (size_t i = 0; i < output->x.size(); i += static_cast<size_t>(sample_step)) {     // 逐采样点绘制车辆附属形状。
      if (state->show_vehicle_outline) {                                                    // 车辆矩形轮廓开关。
        const auto outline = MakeVehicleOutline(output->vehicle_config,
                                                plot_origin,
                                                plot_size,
                                                bounds,
                                                output->x[i],
                                                output->y[i],
                                                output->yaw[i]);
        FillClosedPolygon(draw_list, outline, IM_COL32(180, 100, 255, 72));               // 半透明填充。
      }

      if (state->show_safety_circle) {                                                     // 安全圆开关。
        const double c = std::cos(output->yaw[i]);                                         // 朝向余弦。
        const double s = std::sin(output->yaw[i]);                                         // 朝向正弦。
        const double circle_x =
            output->x[i] + c * vehicle_circle.center_x_body - s * vehicle_circle.center_y_body;  // 车体坐标圆心 -> 世界 x。
        const double circle_y =
            output->y[i] + s * vehicle_circle.center_x_body + c * vehicle_circle.center_y_body;  // 车体坐标圆心 -> 世界 y。
        const auto circle = MakeObstaclePolyline(plot_origin,
                                                 plot_size,
                                                 bounds,
                                                 circle_x,
                                                 circle_y,
                                                 vehicle_circle.radius,
                                                 32);                                       // 安全圆折线采样段数适中。
        DrawPolyline(draw_list, circle, IM_COL32(46, 204, 113, 128), 1.3f);                // 绿色半透明细线。
      }
    }
  }

  const auto start_pt = ToCanvas(plot_origin, plot_size, bounds, preview.x.front(), preview.y.front());  // 起点标记。
  draw_list->AddCircleFilled(start_pt, 4.0f, IM_COL32(255, 255, 255, 255));                                // 白色起点点。
  if (output && !output->x.empty()) {                                                                        // 若有结果，绘制终点标记。
    const auto end_pt = ToCanvas(plot_origin, plot_size, bounds, output->x.back(), output->y.back());
    draw_list->AddCircleFilled(end_pt, 4.0f, IM_COL32(72, 149, 239, 255));
  }

  draw_list->AddText(ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 10.0f),
                     IM_COL32(235, 235, 235, 255),
                     "Reference / initial guess / optimized trajectory");                  // 顶部说明文字。
  char view_label[96];                                                                      // 视图状态文本缓存。
  std::snprintf(view_label, sizeof(view_label), "zoom %.2fx  RMB drag to pan", state->scene_zoom);
  draw_list->AddText(ImVec2(canvas_max.x - 180.0f, canvas_pos.y + 10.0f),
                     IM_COL32(190, 198, 215, 255), view_label);                            // 右上角显示缩放提示。
  const ImVec2 legend_anchor(canvas_pos.x + 14.0f, canvas_max.y - 86.0f);                  // 图例锚点。
  draw_list->AddRectFilled(ImVec2(legend_anchor.x - 6.0f, legend_anchor.y - 6.0f),
                           ImVec2(legend_anchor.x + 170.0f, legend_anchor.y + 74.0f),
                           IM_COL32(10, 12, 18, 170), 6.0f);                                // 图例背景底板。
  DrawLegendEntry(draw_list, legend_anchor, IM_COL32(242, 198, 65, 220), "Reference", state->show_reference_line);
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + 18.0f),
                  IM_COL32(230, 84, 84, 255), "Obstacle", state->show_obstacle);
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + 36.0f),
                  IM_COL32(160, 160, 160, 220), "Initial guess", state->show_initial_guess, true);
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + 54.0f),
                  IM_COL32(72, 149, 239, 255), "Optimized", state->show_optimized_trajectory);
  ImGui::Dummy(canvas_size);                                                                // 吃掉画布空间，防止布局覆盖。
}

// 绘制求解日志面板（总体指标 + outer loop 逐轮摘要）。
void DrawSolverLogPanel(const FrontendState& state) {
  if (!state.has_result) {                                                  // 无结果时给出占位提示。
    ImGui::TextWrapped("No solver log yet.");
    return;
  }

  const PlanningOutput& output = state.result;                              // 结果引用。
  ImGui::Text("Planning time: %.1f ms", output.planning_time_ms);          // 耗时。
  ImGui::Text("Converged: %s", output.converged ? "true" : "false");   // 收敛标志。
  ImGui::Text("Horizon: %d", output.horizon);                              // 时域长度。
  ImGui::Text("Outer iterations: %d", output.outer_iterations);            // 外层迭代次数。
  ImGui::Text("Total inner iterations: %d", output.total_inner_iterations);// 内层累计次数。
  ImGui::Text("Initial cost: %.4f", output.initial_cost);                  // 初始代价。
  ImGui::Text("Final cost: %.4f", output.final_cost);                      // 最终代价。
  ImGui::Text("Initial violation: %.6f", output.initial_violation);        // 初始违反值。
  ImGui::Text("Final violation: %.6f", output.final_violation);            // 最终违反值。

  ImGui::SeparatorText("Outer Loop");                                       // 分组标题。
  ImGui::BeginChild("outer_logs", ImVec2(0.0f, 180.0f), true);             // 固定高度日志滚动区。
  for (const auto& log : output.outer_logs) {                                // 逐条打印每轮摘要。
    ImGui::Text("it=%d inner=%d base=%.3f aug=%.3f viol=%.5f best=%.5f pen=%.1f %s",
                log.outer_iteration,
                log.inner_iterations,
                log.base_cost,
                log.augmented_cost,
                log.max_violation,
                log.best_violation_so_far,
                log.max_penalty,
                log.penalty_updated ? "penalty++" : "");
  }
  ImGui::EndChild();                                                          // 结束日志滚动区。
}

// 构造预览轨迹：使用当前参数生成场景并 rollout 初始控制，
// 让用户在点击 Plan 前先看到“初始猜测”走势。
ScenePreview BuildPreview(const VehicleBicycleConfig& vehicle_config,
                          const StaticObstacleScenarioConfig& scenario_config) {
  ScenePreview preview;                                                       // 返回对象。
  preview.vehicle_config = vehicle_config;                                   // 保存参数快照。
  preview.scenario_config = scenario_config;
  try {
    const auto scenario = my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, scenario_config);  // 生成完整问题实例。
    const auto trajectory =
        scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);  // 用初值控制做前向 rollout。
    preview.x.resize(trajectory.Horizon() + 1);                              // 状态序列长度 N+1。
    preview.y.resize(trajectory.Horizon() + 1);
    preview.yaw.resize(trajectory.Horizon() + 1);
    preview.speed.resize(trajectory.Horizon() + 1);
    for (int k = 0; k <= trajectory.Horizon(); ++k) {                        // 拆解状态分量供 UI 直接绘图。
      preview.x[k] = trajectory.State(k)(0);
      preview.y[k] = trajectory.State(k)(1);
      preview.yaw[k] = trajectory.State(k)(2);
      preview.speed[k] = trajectory.State(k)(3);
    }
    preview.valid = true;                                                     // 标记预览有效。
    preview.status = "Preview updated";                                      // 状态消息。
  } catch (const std::exception& e) {                                         // 捕获构造/rollout 异常。
    preview.status = e.what();                                                // 保留错误文本供 UI 查看。
  }
  return preview;                                                             // 返回预览结果。
}

// 若预览脏标记为 true，则立即刷新预览。
void RefreshPreviewIfNeeded(FrontendState* state) {
  if (!state || !state->preview_dirty) {                                      // 无状态或无需更新则直接返回。
    return;
  }
  state->preview = BuildPreview(state->vehicle_config, state->scenario_config);  // 用最新参数重建预览。
  state->preview_dirty = false;                                               // 清除脏标记。
}

// 后台求解任务：
// - 构建场景
// - 计算初始轨迹
// - 执行 AL-iLQR
// - 写出 CSV
// - 组装日志与可视化序列
PlanningOutput RunPlanningJob(const std::filesystem::path& config_path,
                              const VehicleBicycleConfig& vehicle_config,
                              const StaticObstacleScenarioConfig& scenario_config) {
  PlanningOutput output;                                                      // 输出聚合对象。
  output.vehicle_config = vehicle_config;                                    // 保存参数快照。
  output.scenario_config = scenario_config;
  try {
    const auto t_start = std::chrono::steady_clock::now();                  // 记录开始时间。
    const auto scenario = my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, scenario_config);  // 构造问题。
    const auto initial_trajectory =
        scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);  // 生成初始轨迹。
    ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);  // 构造求解器。
    const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);     // 执行优化。
    const auto t_end = std::chrono::steady_clock::now();                    // 记录结束时间。

    std::filesystem::create_directories("build");                           // 确保输出目录存在。
    output.trajectory_csv_path = "build/imgui_frontend_trajectory.csv";      // 固定 CSV 输出路径。
    my_al_ilqr::WriteTrajectoryCsv(output.trajectory_csv_path, result.trajectory);  // 写轨迹 CSV。

    const int horizon = result.trajectory.Horizon();                         // 提取时域长度。
    output.horizon = horizon;
    output.outer_logs = result.outer_logs;                                   // 拷贝外层日志。
    output.outer_iterations = static_cast<int>(result.outer_logs.size());    // 外层轮数。
    output.total_inner_iterations = 0;                                       // 初始化累计内层次数。
    for (const auto& log : result.outer_logs) {                              // 汇总日志与历史曲线。
      output.total_inner_iterations += log.inner_iterations;
      output.base_cost_history.push_back(log.base_cost);
      output.violation_history.push_back(log.max_violation);
      output.penalty_history.push_back(log.max_penalty);
    }
    output.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count(); // 耗时转毫秒。
    output.initial_cost = scenario.base_problem->TotalCost(initial_trajectory);       // 初始代价。
    output.final_cost = scenario.base_problem->TotalCost(result.trajectory);           // 最终代价。
    output.initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);  // 初始违反。
    output.final_violation = scenario.constrained_problem->MaxViolation(result.trajectory);     // 最终违反。
    output.converged = result.converged;                                     // 收敛状态。
    output.x.resize(horizon + 1);                                            // N+1 状态序列。
    output.y.resize(horizon + 1);
    output.initial_x.resize(horizon + 1);
    output.initial_y.resize(horizon + 1);
    output.yaw.resize(horizon + 1);
    output.speed.resize(horizon + 1);
    output.accel.resize(horizon);                                            // N 控制序列。
    output.kappa.resize(horizon);

    for (int k = 0; k <= horizon; ++k) {                                     // 拆解并复制状态/控制分量。
      output.initial_x[k] = initial_trajectory.State(k)(0);
      output.initial_y[k] = initial_trajectory.State(k)(1);
      output.x[k] = result.trajectory.State(k)(0);
      output.y[k] = result.trajectory.State(k)(1);
      output.yaw[k] = result.trajectory.State(k)(2);
      output.speed[k] = result.trajectory.State(k)(3);
      if (k < horizon) {                                                     // 控制量只有 0..N-1。
        output.accel[k] = result.trajectory.Control(k)(0);                   // 纵向加速度控制。
        output.kappa[k] = std::tan(result.trajectory.Control(k)(1)) / vehicle_config.model.wheelbase;  // 转角 -> 曲率。
      }
    }

    std::ostringstream terminal_state;                                       // 终态文本流。
    terminal_state << result.trajectory.State(horizon).transpose();          // 终态向量转文本。

    output.success = true;                                                   // 标记成功。
    output.terminal_state = terminal_state.str();                            // 保存终态文本。
    output.best_violation = result.best_violation;                           // 最佳违反值。
    std::ostringstream status;                                               // 状态文本流。
    status << "Config: " << config_path.string() << "\n"
           << "Planning time: " << output.planning_time_ms << " ms\n"
           << "Outer iterations: " << output.outer_iterations
           << ", total inner iterations: " << output.total_inner_iterations << "\n"
           << "Initial cost -> final cost: " << output.initial_cost
           << " -> " << output.final_cost << "\n"
           << "Initial violation -> final violation: " << output.initial_violation
           << " -> " << output.final_violation << "\n"
           << "Best violation: " << result.best_violation << "\n"
           << "Trajectory CSV: " << output.trajectory_csv_path.string() << "\n"
           << "Pure ImGui frontend: visualization stays in the main window.";  // 说明该前端为单窗口渲染。
    output.status = status.str();                                            // 保存状态文本。
  } catch (const std::exception& e) {                                        // 捕获任意标准异常。
    output.status = e.what();                                                // 返回错误原因。
  }
  return output;                                                             // 返回任务输出快照。
}

// 初始化默认前端状态（主要是配置路径和车辆参数）。
void SetDefaultState(FrontendState* state) {
  if (!state) {                                                              // 空指针保护。
    return;
  }
  const auto default_config = my_al_ilqr::DefaultVehicleBicycleConfigPath().string();  // 默认配置路径。
  std::snprintf(state->config_path.data(), state->config_path.size(), "%s", default_config.c_str());  // 写入 UI 输入缓冲。
  state->vehicle_config = my_al_ilqr::LoadVehicleBicycleConfig(default_config);          // 按默认路径加载车辆参数。
}

// 从输入框路径加载车辆配置并更新状态文本。
void LoadConfigFromPath(FrontendState* state) {
  if (!state) {                                                              // 空指针保护。
    return;
  }
  try {
    state->vehicle_config = my_al_ilqr::LoadVehicleBicycleConfig(state->config_path.data());  // 加载配置。
    std::ostringstream status;                                               // 组织提示信息。
    status << "Loaded config from " << state->config_path.data();
    state->status = status.str();                                            // 展示加载成功消息。
  } catch (const std::exception& e) {                                        // 捕获读取/解析异常。
    state->status = e.what();                                                // 展示错误原因。
  }
}

// 车辆参数面板：返回是否发生参数变化。
bool DrawVehicleConfigPanel(FrontendState* state) {
  if (!state) {                                                              // 空指针保护。
    return false;
  }
  bool changed = false;                                                      // 收集任意控件变化。
  ImGui::SeparatorText("Vehicle Model");                                    // 分组标题：动力学模型参数。
  changed |= SliderInputDouble("Wheelbase", &state->vehicle_config.model.wheelbase, 1.5, 6.0, "%.2f");
  changed |= SliderInputDouble("Min Accel", &state->vehicle_config.model.min_acceleration, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Max Accel", &state->vehicle_config.model.max_acceleration, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Min Steer", &state->vehicle_config.model.min_steering, -1.2, 1.2, "%.2f");
  changed |= SliderInputDouble("Max Steer", &state->vehicle_config.model.max_steering, -1.2, 1.2, "%.2f");
  changed |= SliderInputDouble("Min Speed", &state->vehicle_config.model.min_speed, -2.0, 20.0, "%.2f");
  changed |= SliderInputDouble("Max Speed", &state->vehicle_config.model.max_speed, -2.0, 20.0, "%.2f");

  ImGui::SeparatorText("Vehicle Body");                                     // 分组标题：几何参数。
  changed |= SliderInputDouble("Length", &state->vehicle_config.body.length, 2.0, 8.0, "%.2f");
  changed |= SliderInputDouble("Width", &state->vehicle_config.body.width, 1.0, 3.0, "%.2f");
  changed |= SliderInputDouble("Rear Axle To Rear", &state->vehicle_config.body.rear_axle_to_rear, 0.2, 3.0, "%.2f");
  return changed;                                                            // 返回是否有改动。
}

// 场景参数面板：返回是否发生参数变化。
bool DrawScenarioPanel(FrontendState* state) {
  if (!state) {                                                              // 空指针保护。
    return false;
  }
  bool changed = false;                                                      // 收集任意控件变化。
  ImGui::SeparatorText("Static Obstacle");                                 // 静态障碍参数组。
  changed |= SliderInputDouble("Obstacle X", &state->scenario_config.obstacle_center_x, 0.0, 20.0, "%.2f");
  changed |= SliderInputDouble("Obstacle Y", &state->scenario_config.obstacle_center_y, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Obstacle Radius", &state->scenario_config.obstacle_radius, 0.1, 5.0, "%.2f");

  ImGui::SeparatorText("Initial State");                                   // 初始状态参数组。
  changed |= SliderInputDouble("Initial X", &state->scenario_config.initial_x, -5.0, 10.0, "%.2f");
  changed |= SliderInputDouble("Initial Y", &state->scenario_config.initial_y, -4.0, 4.0, "%.2f");
  changed |= SliderInputDouble("Initial Yaw", &state->scenario_config.initial_yaw, -3.2, 3.2, "%.2f");
  changed |= SliderInputDouble("Initial Speed", &state->scenario_config.initial_speed, 0.0, 10.0, "%.2f");
  return changed;                                                            // 返回是否有改动。
}

}  // namespace

int main() {
  // 主程序阶段：
  // 1) 初始化默认配置与预览
  // 2) 初始化 GLFW/OpenGL/ImGui
  // 3) 进入事件循环（接收输入、刷新 UI、检查后台求解完成）
  // 4) 退出时清理图形资源
  FrontendState state;                                                       // 前端状态总对象。
  SetDefaultState(&state);                                                   // 加载默认配置。
  RefreshPreviewIfNeeded(&state);                                            // 先构建一次预览。

  if (!glfwInit()) {                                                         // 初始化 GLFW。
    throw std::runtime_error("Failed to initialize GLFW.");                // 失败则抛异常终止。
  }

  const char* glsl_version = "#version 130";                               // OpenGL3 后端使用的 GLSL 版本。
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);                            // 请求 OpenGL 主版本 3。
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);                            // 请求 OpenGL 次版本 0。

  GLFWwindow* window = glfwCreateWindow(1600, 960, "MY-AL-iLQR ImGui Frontend", nullptr, nullptr);  // 创建窗口。
  if (!window) {                                                             // 创建失败处理。
    glfwTerminate();                                                         // 回收 GLFW 资源。
    throw std::runtime_error("Failed to create GLFW window.");             // 抛出异常。
  }

  glfwMakeContextCurrent(window);                                            // 绑定 OpenGL 上下文到当前线程。
  glfwSwapInterval(1);                                                       // 开启垂直同步。

  IMGUI_CHECKVERSION();                                                      // 运行时版本一致性检查。
  ImGui::CreateContext();                                                    // 创建 ImGui 全局上下文。
  ImGuiIO& io = ImGui::GetIO();                                              // 获取 IO 配置引用。
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;                      // 开启键盘导航。
  ImGui::StyleColorsDark();                                                  // 使用暗色主题。

  ImGui_ImplGlfw_InitForOpenGL(window, true);                                // 初始化 GLFW 平台后端。
  ImGui_ImplOpenGL3_Init(glsl_version);                                      // 初始化 OpenGL3 渲染后端。

  std::future<PlanningOutput> planning_future;                               // 保存后台任务 future。

  while (!glfwWindowShouldClose(window)) {                                   // 主循环：直到窗口关闭。
    glfwPollEvents();                                                        // 处理输入与系统事件。

    if (state.planning &&
        planning_future.valid() &&
        planning_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {  // 非阻塞检查后台任务是否完成。
      PlanningOutput output = planning_future.get();                         // 取回任务输出（仅一次）。
      state.planning = false;                                                // 结束规划中状态。
      state.status = output.status;                                          // 更新状态文本。
      state.last_best_violation = output.best_violation;                     // 更新最佳违反值。
      state.terminal_state = output.terminal_state;                          // 更新终态文本。
      state.last_csv_path = output.trajectory_csv_path;                      // 更新 CSV 路径。
      state.has_result = output.success;                                     // 标记是否持有有效结果。
      if (output.success) {                                                  // 成功时才替换完整 result。
        state.result = std::move(output);                                    // 移动赋值减少拷贝。
        state.scene_zoom = 1.0;                                              // 重置视图缩放。
        state.scene_pan_x = 0.0;                                             // 重置平移 x。
        state.scene_pan_y = 0.0;                                             // 重置平移 y。
      }
    }

    ImGui_ImplOpenGL3_NewFrame();                                            // 渲染后端准备新帧。
    ImGui_ImplGlfw_NewFrame();                                               // 平台后端准备新帧。
    ImGui::NewFrame();                                                       // ImGui 新帧开始。
    RefreshPreviewIfNeeded(&state);                                          // 如有必要刷新预览。

    const ImGuiViewport* viewport = ImGui::GetMainViewport();                // 主视口信息。
    const float controls_width = 340.0f;                                     // 左侧控制面板宽度。
    const float content_padding = 16.0f;                                     // 全局面板间距。
    const float right_x = viewport->Pos.x + controls_width + content_padding;  // 右侧区域起始 x。
    const float right_width = viewport->Size.x - controls_width - 2.0f * content_padding;  // 右侧区域宽度。
    const float top_height = viewport->Size.y * 0.60f;                       // 右上可视化面板高度。
    const float bottom_height = viewport->Size.y - top_height - 3.0f * content_padding;  // 右下调试面板高度。
    const float bottom_third_width = (right_width - 2.0f * content_padding) / 3.0f;  // 预留的三等分宽（保留变量，不改变原行为）。
    (void)bottom_third_width;                                                // 显式抑制未使用警告，语义不变。

    ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x + content_padding, viewport->Pos.y + content_padding),
                            ImGuiCond_Always);                               // 固定 Controls 窗口位置。
    ImGui::SetNextWindowSize(ImVec2(controls_width - content_padding, viewport->Size.y - 2.0f * content_padding),
                             ImGuiCond_Always);                              // 固定 Controls 窗口尺寸。
    ImGui::Begin("Controls");                                               // 左侧控制面板开始。
    ImGui::InputText("Config Path", state.config_path.data(), state.config_path.size());  // 配置路径输入框。
    if (ImGui::Button("Load Config")) {                                    // 点击加载配置。
      LoadConfigFromPath(&state);
    }
    ImGui::SameLine();                                                       // 与 Load 按钮同行放置 Plan。
    const bool disable_plan_button = state.planning;                         // 规划中禁用 Plan 防重复提交。
    if (disable_plan_button) {
      ImGui::BeginDisabled();                                                // 开始禁用域。
    }
    if (ImGui::Button("Plan")) {
      // 点击 Plan 后将重计算任务投递到后台线程，
      // UI 线程保持响应，避免长时间阻塞界面交互。
      state.status = "Planning...";                                         // 即时反馈。
      state.planning = true;                                                 // 标记规划中。
      const std::filesystem::path config_path = state.config_path.data();    // 捕获当前路径快照。
      const VehicleBicycleConfig vehicle_config = state.vehicle_config;       // 捕获当前车辆参数快照。
      const StaticObstacleScenarioConfig scenario_config = state.scenario_config;  // 捕获当前场景参数快照。
      planning_future = std::async(std::launch::async,
                                   [config_path, vehicle_config, scenario_config]() {
                                     return RunPlanningJob(config_path, vehicle_config, scenario_config);  // 后台执行规划。
                                   });
    }
    if (disable_plan_button) {
      ImGui::EndDisabled();                                                  // 结束禁用域。
    }

    ImGui::Separator();                                                      // 分隔线。
    ImGui::TextWrapped("Status: %s", state.status.c_str());                 // 状态文本（自动换行）。
    ImGui::BulletText("Best violation: %.6f", state.last_best_violation);   // 最佳违反值。
    ImGui::BulletText("Trajectory CSV: %s",
                      state.last_csv_path.empty() ? "not written" : state.last_csv_path.string().c_str());  // CSV 路径提示。
    ImGui::BulletText("Terminal state: %s",
                      state.terminal_state.empty() ? "not available" : state.terminal_state.c_str());       // 终态提示。
    ImGui::SeparatorText("Solver Logs");                                    // 求解日志分组标题。
    DrawSolverLogPanel(state);                                               // 绘制日志面板内容。

    bool params_changed = false;                                             // 跟踪参数区是否有修改。
    params_changed |= DrawVehicleConfigPanel(&state);                        // 车辆参数面板。
    params_changed |= DrawScenarioPanel(&state);                             // 场景参数面板。
    if (params_changed) {
      // 参数变更后立即刷新 preview，并清空旧求解结果，
      // 避免“新参数 + 旧结果”混用造成误判。
      state.preview_dirty = true;                                            // 标记预览需重建。
      state.has_result = false;                                              // 旧优化结果失效。
      state.scene_zoom = 1.0;                                                // 视角重置。
      state.scene_pan_x = 0.0;
      state.scene_pan_y = 0.0;
      RefreshPreviewIfNeeded(&state);                                        // 立即更新预览，提升反馈速度。
    }
    ImGui::End();                                                            // Controls 面板结束。

    ImGui::SetNextWindowPos(ImVec2(right_x, viewport->Pos.y + content_padding), ImGuiCond_Always);  // Visualization 窗口位置。
    ImGui::SetNextWindowSize(ImVec2(right_width, top_height - content_padding), ImGuiCond_Always);  // Visualization 窗口大小。
    ImGui::Begin("Visualization");                                          // 可视化面板开始。
    ImGui::BeginGroup();                                                     // 左侧交互组。
    if (ImGui::Button("Reset View")) {                                     // 视角复位按钮。
      state.scene_zoom = 1.0;
      state.scene_pan_x = 0.0;
      state.scene_pan_y = 0.0;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Mouse wheel: zoom   Right-drag: pan");         // 操作提示。
    ImGui::EndGroup();
    ImGui::SameLine(ImGui::GetWindowWidth() - 270.0f);                       // 右侧开关组靠右对齐。
    ImGui::BeginGroup();
    ImGui::Checkbox("Ref", &state.show_reference_line);                    // 参考线开关。
    ImGui::SameLine();
    ImGui::Checkbox("Obstacle", &state.show_obstacle);                     // 障碍物开关。
    ImGui::SameLine();
    ImGui::Checkbox("Init", &state.show_initial_guess);                    // 初始轨迹开关。
    ImGui::SameLine();
    ImGui::Checkbox("Opt", &state.show_optimized_trajectory);              // 优化轨迹开关。
    ImGui::Checkbox("Rect", &state.show_vehicle_outline);                  // 车辆矩形开关。
    ImGui::SameLine();
    ImGui::Checkbox("Circle", &state.show_safety_circle);                  // 安全圆开关。
    ImGui::EndGroup();
    ImGui::Separator();
    if (state.has_result) {                                                  // 有结果时绘制结果视图。
      DrawSceneView(&state);
    } else if (state.preview.valid) {                                        // 无结果但有预览时绘制预览视图。
      DrawSceneView(&state);
    } else {
      ImGui::TextWrapped("No preview available.");                          // 无任何数据时提示。
    }
    ImGui::End();                                                            // Visualization 面板结束。

    ImGui::SetNextWindowPos(ImVec2(right_x, viewport->Pos.y + top_height + content_padding), ImGuiCond_Always);  // Debug 窗口位置。
    ImGui::SetNextWindowSize(ImVec2(right_width, bottom_height), ImGuiCond_Always);                               // Debug 窗口大小。
    ImGui::Begin("Debug");                                                  // 调试曲线面板开始。
    ImGui::Columns(3, "debug_columns_top", false);                          // 顶部 3 列：速度/曲率/加速度。
    DrawSignalPlot("Speed", state.has_result ? state.result.speed : std::vector<double>{},
                   IM_COL32(74, 201, 120, 255),
                   state.vehicle_config.model.min_speed,
                   state.vehicle_config.model.max_speed);
    ImGui::NextColumn();
    DrawSignalPlot("Kappa", state.has_result ? state.result.kappa : std::vector<double>{},
                   IM_COL32(83, 131, 255, 255));
    ImGui::NextColumn();
    DrawSignalPlot("Acceleration", state.has_result ? state.result.accel : std::vector<double>{},
                   IM_COL32(255, 177, 66, 255),
                   state.vehicle_config.model.min_acceleration,
                   state.vehicle_config.model.max_acceleration);
    ImGui::Columns(1);                                                       // 恢复单列。
    ImGui::Separator();
    ImGui::Columns(3, "debug_columns_bottom", false);                       // 底部 3 列：代价/违反/罚参数。
    DrawSignalPlot("Base Cost",
                   state.has_result ? state.result.base_cost_history : std::vector<double>{},
                   IM_COL32(255, 120, 120, 255));
    ImGui::NextColumn();
    DrawSignalPlot("Violation",
                   state.has_result ? state.result.violation_history : std::vector<double>{},
                   IM_COL32(255, 210, 90, 255),
                   0.0);
    ImGui::NextColumn();
    DrawSignalPlot("Penalty",
                   state.has_result ? state.result.penalty_history : std::vector<double>{},
                   IM_COL32(180, 120, 255, 255),
                   0.0);
    ImGui::Columns(1);                                                       // 恢复单列。
    ImGui::End();                                                            // Debug 面板结束。

    ImGui::Render();                                                         // 生成 ImGui 绘制命令。
    int display_w = 0;                                                       // 帧缓冲宽。
    int display_h = 0;                                                       // 帧缓冲高。
    glfwGetFramebufferSize(window, &display_w, &display_h);                 // 读取当前像素尺寸（HiDPI 友好）。
    glViewport(0, 0, display_w, display_h);                                 // 设置 OpenGL 视口。
    glClearColor(0.08f, 0.09f, 0.12f, 1.0f);                                // 背景清屏色。
    glClear(GL_COLOR_BUFFER_BIT);                                           // 清空颜色缓冲。
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());                 // 提交 ImGui 绘制数据到 OpenGL。
    glfwSwapBuffers(window);                                                // 前后缓冲交换，显示本帧。
  }

  if (planning_future.valid()) {                                            // 退出前若仍有后台任务，等待其结束。
    planning_future.wait();
  }

  ImGui_ImplOpenGL3_Shutdown();                                             // 关闭 ImGui OpenGL 后端。
  ImGui_ImplGlfw_Shutdown();                                                // 关闭 ImGui GLFW 后端。
  ImGui::DestroyContext();                                                  // 销毁 ImGui 上下文。
  glfwDestroyWindow(window);                                                // 销毁窗口。
  glfwTerminate();                                                          // 终止 GLFW。
  return 0;                                                                 // 正常退出。
}
