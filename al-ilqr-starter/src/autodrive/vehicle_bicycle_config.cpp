#include "autodrive/vehicle_bicycle_config.hpp"

#include <cmath>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace my_al_ilqr {

namespace {

// INI 解析后使用的二级字典结构：
// values[section][key] = value_string
using SectionValues = std::unordered_map<std::string, std::string>;
using IniValues = std::unordered_map<std::string, SectionValues>;

// 去除字符串首尾空白字符，避免配置中多余空格影响解析。
std::string Trim(const std::string& input) {
  std::size_t first = 0;
  while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first])) != 0) {
    ++first;
  }
  std::size_t last = input.size();
  while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1])) != 0) {
    --last;
  }
  return input.substr(first, last - first);
}

// 读取并解析简化 INI 文件。
// 支持 [section] 与 key=value，忽略空行及 #/; 注释。
IniValues ParseIniFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open vehicle config file: " + path.string());
  }

  IniValues values;
  std::string line;
  std::string current_section;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed.front() == '#' || trimmed.front() == ';') {
      continue;
    }
    if (trimmed.front() == '[') {
      if (trimmed.back() != ']') {
        throw std::runtime_error("Invalid section header in vehicle config at line " +
                                 std::to_string(line_number));
      }
      current_section = Trim(trimmed.substr(1, trimmed.size() - 2));
      continue;
    }

    const std::size_t equal_pos = trimmed.find('=');
    if (equal_pos == std::string::npos) {
      throw std::runtime_error("Invalid key/value pair in vehicle config at line " +
                               std::to_string(line_number));
    }
    if (current_section.empty()) {
      throw std::runtime_error("Vehicle config key/value pair found before any section.");
    }

    const std::string key = Trim(trimmed.substr(0, equal_pos));
    const std::string value = Trim(trimmed.substr(equal_pos + 1));
    values[current_section][key] = value;
  }

  return values;
}

// 从解析后的 INI 中提取必填 double，并在缺失/格式错误时抛异常。
double GetRequiredDouble(const IniValues& values,
                         const std::string& section,
                         const std::string& key) {
  const auto section_it = values.find(section);
  if (section_it == values.end()) {
    throw std::runtime_error("Vehicle config missing section [" + section + "].");
  }
  const auto value_it = section_it->second.find(key);
  if (value_it == section_it->second.end()) {
    throw std::runtime_error("Vehicle config missing key [" + section + "] " + key + ".");
  }
  try {
    return std::stod(value_it->second);
  } catch (const std::exception&) {
    throw std::runtime_error("Vehicle config has invalid floating-point value for [" + section +
                             "] " + key + ".");
  }
}

// 对配置做物理与几何一致性检查，避免求解阶段出现不可解释行为。
void ValidateConfig(const VehicleBicycleConfig& config) {
  if (config.model.wheelbase <= 0.0) {
    throw std::runtime_error("Vehicle config wheelbase must be positive.");
  }
  if (config.model.min_acceleration > config.model.max_acceleration) {
    throw std::runtime_error("Vehicle config acceleration bounds are inconsistent.");
  }
  if (config.model.min_steering > config.model.max_steering) {
    throw std::runtime_error("Vehicle config steering bounds are inconsistent.");
  }
  if (config.model.min_speed > config.model.max_speed) {
    throw std::runtime_error("Vehicle config speed bounds are inconsistent.");
  }
  if (config.body.length <= 0.0 || config.body.width <= 0.0) {
    throw std::runtime_error("Vehicle body length and width must be positive.");
  }
  if (config.body.rear_axle_to_rear < 0.0 || config.body.rear_axle_to_rear >= config.body.length) {
    throw std::runtime_error("Vehicle rear axle position must lie inside the body length.");
  }
}

}  // namespace

std::filesystem::path DefaultVehicleBicycleConfigPath() {
  // 默认配置跟随源码目录，便于示例程序“开箱即用”。
  return std::filesystem::path(MY_AL_ILQR_SOURCE_DIR) / "config" / "vehicle_bicycle_config.ini";
}

VehicleBicycleConfig LoadVehicleBicycleConfig(const std::filesystem::path& path) {
  // 数据流：ini 文本 -> 字典 -> 结构化 VehicleBicycleConfig -> 合法性验证。
  const IniValues values = ParseIniFile(path);

  VehicleBicycleConfig config;
  config.model.wheelbase = GetRequiredDouble(values, "bicycle_model", "wheelbase");
  config.model.min_acceleration =
      GetRequiredDouble(values, "bicycle_model", "min_acceleration");
  config.model.max_acceleration =
      GetRequiredDouble(values, "bicycle_model", "max_acceleration");
  config.model.min_steering = GetRequiredDouble(values, "bicycle_model", "min_steering");
  config.model.max_steering = GetRequiredDouble(values, "bicycle_model", "max_steering");
  config.model.min_speed = GetRequiredDouble(values, "bicycle_model", "min_speed");
  config.model.max_speed = GetRequiredDouble(values, "bicycle_model", "max_speed");

  config.body.length = GetRequiredDouble(values, "vehicle_body", "length");
  config.body.width = GetRequiredDouble(values, "vehicle_body", "width");
  config.body.rear_axle_to_rear =
      GetRequiredDouble(values, "vehicle_body", "rear_axle_to_rear");

  ValidateConfig(config);
  return config;
}

VehicleCollisionCircle BuildSingleCircleVehicleApproximation(const VehicleBodyConfig& config) {
  if (config.length <= 0.0 || config.width <= 0.0) {
    throw std::invalid_argument("Single-circle vehicle approximation requires positive body dimensions.");
  }
  if (config.rear_axle_to_rear < 0.0 || config.rear_axle_to_rear >= config.length) {
    throw std::invalid_argument("Single-circle vehicle approximation requires a valid rear axle offset.");
  }

  // 前后方向几何量（以后轴中心为原点，x 朝车头）。
  const double rear = config.rear_axle_to_rear;
  const double front = config.length - config.rear_axle_to_rear;

  VehicleCollisionCircle circle;
  // 单圆中心取矩形车体前后中点在后轴坐标系下的位置。
  circle.center_x_body = 0.5 * (front - rear);
  circle.center_y_body = 0.0;
  // 半径取能覆盖整车矩形的外接圆半径：0.5 * sqrt(length^2 + width^2)。
  circle.radius = 0.5 * std::sqrt(config.length * config.length + config.width * config.width);
  return circle;
}

}  // namespace my_al_ilqr
