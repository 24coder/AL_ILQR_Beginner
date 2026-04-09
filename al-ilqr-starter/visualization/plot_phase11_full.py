#!/usr/bin/env python3
"""
phase11 完整仿真可视化脚本。

用途：
  对 phase11_dynamic_obstacle_demo 产生的 4 个 CSV 文件进行全面可视化：
  - 初始轨迹 vs 优化轨迹（几何场景）
  - 动态障碍物预测路径与每时刻位置
  - 车辆姿态（车体矩形采样）
  - 速度 / 加速度 / 转向控制时序
  - 外层 AL 迭代收敛日志（代价、违反量、罚参数）

使用方法：
  python3 visualization/plot_phase11_full.py

  或直接在项目根目录执行（脚本自动用 build/ 下的默认路径）：
  python3 visualization/plot_phase11_full.py \\
      build/phase11_dynamic_initial_trajectory.csv \\
      build/phase11_dynamic_optimized_trajectory.csv \\
      build/phase11_dynamic_obstacle_prediction.csv \\
      build/phase11_dynamic_outer_log.csv
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle, Polygon, FancyArrow
from matplotlib.lines import Line2D


# ──────────────────────────────────────────────
# 场景几何常量（与 demo_scenario.hpp 保持一致）
# ──────────────────────────────────────────────
ROAD_LOWER = -2.3      # 道路下边界 [m]
ROAD_UPPER = 2.3       # 道路上边界 [m]
VEHICLE_LENGTH = 4.6   # 车长 [m]
VEHICLE_WIDTH = 1.9    # 车宽 [m]
REAR_AXLE_TO_REAR = 1.0  # 后轴到车尾距离 [m]（用于车体矩形定位）


# ──────────────────────────────────────────────
# 辅助：读取 CSV
# ──────────────────────────────────────────────
def load_csv(path: Path) -> list:
    """将 CSV 文件读为字典列表，每行是一个 dict，key 为列名。"""
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


# ──────────────────────────────────────────────
# 辅助：绘制车体矩形
# ──────────────────────────────────────────────
def draw_vehicle(ax, x, y, yaw, color="tab:blue", alpha=0.20, zorder=3):
    """
    在坐标 (x, y) 处以航向角 yaw 绘制车体矩形轮廓。

    采用后轴为参考点（x, y 表示后轴中心），
    矩形在车体坐标系下的范围为：
      前方 +（length - rear_axle_to_rear）
      后方 -rear_axle_to_rear
      左右 ±width/2

    这样绘制的矩形比"几何中心"更接近 bicycle model 的实际位置。
    """
    half_w = VEHICLE_WIDTH / 2.0
    front = VEHICLE_LENGTH - REAR_AXLE_TO_REAR
    rear = -REAR_AXLE_TO_REAR

    # 四个角点在车体坐标系下（前后 × 左右）
    corners_body = [
        (front, half_w),
        (front, -half_w),
        (rear, -half_w),
        (rear, half_w),
    ]

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    # 旋转到世界坐标系
    corners_world = [
        (x + cos_y * cx - sin_y * cy,
         y + sin_y * cx + cos_y * cy)
        for cx, cy in corners_body
    ]

    patch = Polygon(corners_world, closed=True,
                    facecolor=color, edgecolor=color,
                    alpha=alpha, zorder=zorder)
    ax.add_patch(patch)

    # 绘制车头方向箭头（短线，指示朝向）
    arrow_len = 0.9
    ax.annotate("",
                xy=(x + cos_y * front * 0.75, y + sin_y * front * 0.75),
                xytext=(x, y),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.2, alpha=min(1.0, alpha * 2.5)),
                zorder=zorder + 1)


# ──────────────────────────────────────────────
# 辅助：时间步索引对应时刻
# ──────────────────────────────────────────────
def k_to_time(k, dt=0.1):
    return k * dt


# ──────────────────────────────────────────────
# 主绘图函数
# ──────────────────────────────────────────────
def main():
    # ── 默认路径（在项目根目录运行时自动匹配）──
    default_initial  = Path("build/phase11_dynamic_initial_trajectory.csv")
    default_optimized = Path("build/phase11_dynamic_optimized_trajectory.csv")
    default_obstacle  = Path("build/phase11_dynamic_obstacle_prediction.csv")
    default_outer_log = Path("build/phase11_dynamic_outer_log.csv")

    if len(sys.argv) == 5:
        initial_csv   = Path(sys.argv[1])
        optimized_csv = Path(sys.argv[2])
        obstacle_csv  = Path(sys.argv[3])
        outer_log_csv = Path(sys.argv[4])
    elif len(sys.argv) == 1:
        initial_csv   = default_initial
        optimized_csv = default_optimized
        obstacle_csv  = default_obstacle
        outer_log_csv = default_outer_log
    else:
        print(
            "Usage:\n"
            "  python3 visualization/plot_phase11_full.py\n"
            "  python3 visualization/plot_phase11_full.py \\\n"
            "      build/phase11_dynamic_initial_trajectory.csv \\\n"
            "      build/phase11_dynamic_optimized_trajectory.csv \\\n"
            "      build/phase11_dynamic_obstacle_prediction.csv \\\n"
            "      build/phase11_dynamic_outer_log.csv"
        )
        sys.exit(1)

    # ── 读取数据 ──────────────────────────────────
    print("Loading CSVs...")
    init_rows     = load_csv(initial_csv)
    opt_rows      = load_csv(optimized_csv)
    obs_rows      = load_csv(obstacle_csv)
    log_rows      = load_csv(outer_log_csv)

    # 初始轨迹字段：k, x0(px), x1(py), x2(yaw), x3(speed)
    x_init  = [float(r["x0"]) for r in init_rows]
    y_init  = [float(r["x1"]) for r in init_rows]
    yaw_init = [float(r["x2"]) for r in init_rows]

    # 优化轨迹字段：k, x0, x1, x2, x3, u0(accel), u1(steer)
    x_opt   = [float(r["x0"]) for r in opt_rows]
    y_opt   = [float(r["x1"]) for r in opt_rows]
    yaw_opt = [float(r["x2"]) for r in opt_rows]
    spd_opt = [float(r["x3"]) for r in opt_rows]
    k_opt   = [int(r["k"]) for r in opt_rows]
    t_opt   = [k_to_time(k) for k in k_opt]
    # 控制量只在 k=0..N-1 有效，最后一行 u 为空（终端状态无控制）
    accel_opt = [float(r["u0"]) if r.get("u0", "") != "" else float("nan") for r in opt_rows]
    steer_opt = [float(r["u1"]) if r.get("u1", "") != "" else float("nan") for r in opt_rows]

    # 障碍物预测：k, cx, cy, radius
    obs_x  = [float(r["cx"])     for r in obs_rows]
    obs_y  = [float(r["cy"])     for r in obs_rows]
    obs_r  = float(obs_rows[0]["radius"])
    obs_k  = [int(r["k"])        for r in obs_rows]
    obs_t  = [k_to_time(k) for k in obs_k]

    # 外层 AL 日志
    outer_iter    = [int(r["outer_iteration"])    for r in log_rows]
    base_cost     = [float(r["base_cost"])        for r in log_rows]
    aug_cost      = [float(r["augmented_cost"])   for r in log_rows]
    violation     = [float(r["max_violation"])    for r in log_rows]
    best_viol     = [float(r["best_violation_so_far"]) for r in log_rows]
    penalty       = [float(r["max_penalty"])      for r in log_rows]
    pen_updated   = [r["penalty_updated"].strip() == "1" for r in log_rows]

    # ── 布局设计 ─────────────────────────────────
    # 共 3 行 × 3 列。
    # 第 0 行：整行大宽图 → 几何场景（轨迹 + 障碍物）
    # 第 1 行：三列 → 速度曲线 | 控制曲线 | 障碍物 y 随时间
    # 第 2 行：三列 → AL 代价收敛 | AL 违反量收敛 | AL 罚参数
    fig = plt.figure(figsize=(16, 13))
    fig.suptitle("Phase 11 – Dynamic Obstacle Avoidance (AL-iLQR)", fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(3, 3, figure=fig,
                           height_ratios=[1.4, 1.0, 1.0],
                           hspace=0.45, wspace=0.35)

    ax_scene   = fig.add_subplot(gs[0, :])      # 第 0 行整行：几何场景
    ax_speed   = fig.add_subplot(gs[1, 0])      # 速度
    ax_ctrl    = fig.add_subplot(gs[1, 1])      # 控制
    ax_obs_y   = fig.add_subplot(gs[1, 2])      # 障碍物 y 轨迹
    ax_cost    = fig.add_subplot(gs[2, 0])      # AL 代价
    ax_viol    = fig.add_subplot(gs[2, 1])      # AL 违反量
    ax_pen     = fig.add_subplot(gs[2, 2])      # AL 罚参数

    # ════════════════════════════════════════════
    # 图 1：几何场景图
    # ════════════════════════════════════════════
    ax = ax_scene

    # 道路背景填充（浅色）
    ax.axhspan(ROAD_LOWER, ROAD_UPPER, color="lightyellow", alpha=0.6, zorder=0)

    # 道路边界线
    ax.axhline(ROAD_UPPER, color="darkorange", linestyle="--", linewidth=1.5, label="road boundary")
    ax.axhline(ROAD_LOWER, color="darkorange", linestyle="--", linewidth=1.5)

    # 参考线（x 轴 y=0 的直线）
    ax.axhline(0.0, color="deepskyblue", linestyle=":", linewidth=1.8, label="reference line", zorder=1)

    # 初始轨迹（灰色虚线）
    ax.plot(x_init, y_init, linestyle="--", color="silver", linewidth=2.0,
            label="initial trajectory", zorder=2)

    # 优化轨迹（蓝色实线）
    ax.plot(x_opt, y_opt, color="royalblue", linewidth=2.5,
            label="optimized trajectory", zorder=3)

    # 障碍物预测路径（红色细线）
    ax.plot(obs_x, obs_y, color="tomato", linewidth=1.5, linestyle="-",
            label="obstacle predicted path", zorder=4, alpha=0.85)

    # 障碍物圆（按时刻采样，透明度随时间递增表示"晚时刻更不确定"）
    n_obs = len(obs_rows)
    sample_obs_ids = sorted(set([
        0,
        n_obs // 5,
        n_obs * 2 // 5,
        n_obs * 3 // 5,
        n_obs * 4 // 5,
        n_obs - 1,
    ]))
    for idx in sample_obs_ids:
        t_frac = idx / max(1, n_obs - 1)
        alpha_circle = 0.08 + 0.22 * t_frac   # 早期透明，晚期更不透明
        label_circle = f"obstacle (t={obs_t[idx]:.1f}s)" if idx == sample_obs_ids[-1] else None
        circ = Circle((obs_x[idx], obs_y[idx]), obs_r,
                      color="tomato", alpha=alpha_circle, zorder=4)
        ax.add_patch(circ)
        # 在每个障碍圆心处标记时刻
        ax.text(obs_x[idx] + obs_r + 0.05, obs_y[idx],
                f"t={obs_t[idx]:.1f}s", fontsize=7, color="darkred", va="center", zorder=6)

    # 在优化轨迹上采样绘制车体矩形
    n_opt = len(opt_rows)
    sample_opt_ids = sorted(set([
        0,
        n_opt // 5,
        n_opt * 2 // 5,
        n_opt * 3 // 5,
        n_opt - 1,
    ]))
    for idx in sample_opt_ids:
        draw_vehicle(ax, x_opt[idx], y_opt[idx], yaw_opt[idx],
                     color="royalblue", alpha=0.20, zorder=5)

    # 在初始轨迹上抽样绘制车体（更淡）
    for idx in [0, n_opt // 2, n_opt - 1]:
        draw_vehicle(ax, x_init[idx], y_init[idx], yaw_init[idx],
                     color="gray", alpha=0.10, zorder=4)

    # 标注起点和终点
    ax.scatter([x_opt[0]], [y_opt[0]], color="green", s=80, zorder=7, label="start")
    ax.scatter([x_opt[-1]], [y_opt[-1]], color="navy", s=80, marker="*", zorder=7, label="goal")

    ax.set_title("Geometric Scene: Trajectory vs Dynamic Obstacle", fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.85)

    # ════════════════════════════════════════════
    # 图 2：速度时序（优化结果）
    # ════════════════════════════════════════════
    ax = ax_speed
    ax.plot(t_opt, spd_opt, color="forestgreen", linewidth=2.0, label="speed")
    # 速度边界参考线（demo scenario 里 min/max speed 通常为 0 ~ 5）
    ax.axhline(0.0, color="gray", linestyle=":", linewidth=1.0)
    ax.axhline(5.0, color="red",  linestyle=":", linewidth=1.0, alpha=0.5, label="max speed limit")
    ax.fill_between(t_opt, spd_opt, 0, alpha=0.10, color="forestgreen")
    ax.set_title("Optimized Speed Profile", fontsize=10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=8)

    # ════════════════════════════════════════════
    # 图 3：控制输入时序（加速度 + 转向）
    # ════════════════════════════════════════════
    ax = ax_ctrl
    # 过滤掉末尾 nan（终端状态没有控制量）
    t_ctrl    = [t for t, a in zip(t_opt, accel_opt) if not math.isnan(a)]
    accel_plt = [a for a in accel_opt if not math.isnan(a)]
    steer_plt = [s for s in steer_opt if not math.isnan(s)]

    ax.plot(t_ctrl, accel_plt, color="tomato",  linewidth=2.0, label="accel [m/s²]")
    ax.plot(t_ctrl, steer_plt, color="purple",  linewidth=2.0, label="steer [rad]")

    # 控制约束边界（vehicle_config 默认值）
    ax.axhline( 2.0, color="tomato",  linestyle=":", alpha=0.5, linewidth=1.0)
    ax.axhline(-3.0, color="tomato",  linestyle=":", alpha=0.5, linewidth=1.0)
    ax.axhline( 0.6, color="purple",  linestyle=":", alpha=0.5, linewidth=1.0)
    ax.axhline(-0.6, color="purple",  linestyle=":", alpha=0.5, linewidth=1.0)
    ax.axhline(0.0,  color="gray",    linestyle="--", linewidth=0.8)

    ax.set_title("Optimized Controls", fontsize=10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("control value")
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=8)

    # ════════════════════════════════════════════
    # 图 4：障碍物 y 方向随时间的运动
    #        vs 车辆 y 方向随时间的轨迹
    #        帮助直觉判断"车辆是否在时空上错开了障碍"
    # ════════════════════════════════════════════
    ax = ax_obs_y
    ax.plot(obs_t, obs_y, color="tomato", linewidth=2.0, linestyle="-", label="obstacle cy(t)")
    ax.fill_between(obs_t,
                    [cy - obs_r for cy in obs_y],
                    [cy + obs_r for cy in obs_y],
                    color="tomato", alpha=0.12, label=f"obstacle radius ±{obs_r}m")

    # 车辆 y(t) 曲线
    ax.plot(t_opt, y_opt, color="royalblue", linewidth=2.0, label="vehicle y(t)")

    ax.axhline(ROAD_UPPER, color="darkorange", linestyle="--", linewidth=1.0, alpha=0.7, label="road boundary")
    ax.axhline(ROAD_LOWER, color="darkorange", linestyle="--", linewidth=1.0, alpha=0.7)
    ax.axhline(0.0, color="deepskyblue", linestyle=":", linewidth=1.0, alpha=0.7, label="reference")

    ax.set_title("Lateral Motion vs Obstacle (time domain)", fontsize=10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=8, loc="upper left")

    # ════════════════════════════════════════════
    # 图 5：AL 代价收敛（base cost + augmented cost）
    # ════════════════════════════════════════════
    ax = ax_cost
    ax.plot(outer_iter, base_cost, marker="o", color="royalblue", linewidth=2.0, label="base cost")
    ax.plot(outer_iter, aug_cost,  marker="s", color="gray",      linewidth=1.5,
            linestyle="--", label="augmented cost", alpha=0.7)
    # 标注 penalty 更新的轮次
    for i, (it, updated) in enumerate(zip(outer_iter, pen_updated)):
        if updated:
            ax.axvline(it, color="orange", linestyle=":", linewidth=1.2, alpha=0.8)
    ax.set_title("AL Outer Loop: Cost", fontsize=10)
    ax.set_xlabel("outer iteration")
    ax.set_ylabel("cost")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=8)
    # 橙色竖线图例
    ax.plot([], [], color="orange", linestyle=":", linewidth=1.2, label="penalty updated")
    ax.legend(fontsize=8)

    # ════════════════════════════════════════════
    # 图 6：AL 违反量收敛
    # ════════════════════════════════════════════
    ax = ax_viol
    ax.plot(outer_iter, violation,  marker="o", color="tomato",  linewidth=2.0, label="max violation")
    ax.plot(outer_iter, best_viol,  marker="s", color="green",   linewidth=1.5,
            linestyle="--", label="best violation so far")
    # 约束容差参考线（从 demo 参数中取 0.1）
    ax.axhline(0.1, color="darkgreen", linestyle=":", linewidth=1.5, label="constraint tol=0.1")
    for it, updated in zip(outer_iter, pen_updated):
        if updated:
            ax.axvline(it, color="orange", linestyle=":", linewidth=1.2, alpha=0.8)
    ax.set_title("AL Outer Loop: Constraint Violation", fontsize=10)
    ax.set_xlabel("outer iteration")
    ax.set_ylabel("max violation")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.4, which="both")
    ax.legend(fontsize=8)

    # ════════════════════════════════════════════
    # 图 7：AL 罚参数随外层迭代的演化
    # ════════════════════════════════════════════
    ax = ax_pen
    ax.step(outer_iter, penalty, color="darkorange", linewidth=2.0, where="post", label="max penalty")
    # 在 penalty 更新处标注
    for it, updated, pen in zip(outer_iter, pen_updated, penalty):
        if updated:
            ax.scatter([it], [pen], color="red", s=50, zorder=5)
    ax.set_title("AL Outer Loop: Penalty Parameter", fontsize=10)
    ax.set_xlabel("outer iteration")
    ax.set_ylabel("penalty (μ)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.4, which="both")

    # 自定义图例：红点 = penalty 被更新的那一轮
    legend_elements = [
        Line2D([0], [0], color="darkorange", linewidth=2, label="max penalty"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="red",
               markersize=8, label="penalty updated"),
    ]
    ax.legend(handles=legend_elements, fontsize=8)

    # ── 保存图片 ──────────────────────────────────
    output_path = optimized_csv.parent / "phase11_full_visualization.png"
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    print(f"Saved full visualization to: {output_path}")
    plt.show()


if __name__ == "__main__":
    main()
