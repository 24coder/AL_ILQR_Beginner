#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

// 直线参考线。
// 以 (origin_x, origin_y) 为参考原点，heading 为参考方向角。
// 可将全局坐标下状态投影到 Frenet 风格的纵向/横向误差坐标。
class StraightReferenceLine {
 public:
  StraightReferenceLine(double origin_x = 0.0,
                       double origin_y = 0.0,
                       double heading = 0.0);

  // 纵向投影 s：状态位置在参考线方向上的坐标。
  double LongitudinalPosition(const Vector& state) const;
  // 横向误差 e_y：状态到参考线的有符号垂向偏移。
  double LateralError(const Vector& state) const;
  // 航向误差 e_psi：车辆航向与参考线方向差值。
  double HeadingError(const Vector& state) const;

  double OriginX() const { return origin_x_; }
  double OriginY() const { return origin_y_; }
  double Heading() const { return heading_; }

 private:
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double heading_ = 0.0;
};

}  // namespace my_al_ilqr
