#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


# 统一读取 CSV。
# summary CSV 字段假设：
# - obstacle_speed: 障碍物速度扫描值
# - response: 文本标签（如 brake / evade 等策略结果）
# - crossing_delay: 与基线相比的通过延迟指标
# - max_abs_lateral: 最大横向偏移绝对值
def load_csv(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


# 从轨迹 CSV 抽取平面路径。
# 轨迹文件字段假设至少包含 x0, x1（车辆 x-y 位置）。
def load_traj(path: Path):
    rows = load_csv(path)
    return [float(row["x0"]) for row in rows], [float(row["x1"]) for row in rows]


def main():
    if len(sys.argv) != 5:
        print(
            "Usage: python3 visualization/plot_phase12_speed_sweep.py "
            "build/phase12_dynamic_speed_sweep_summary.csv "
            "build/phase12_speed_0p25_optimized_trajectory.csv "
            "build/phase12_speed_1p15_optimized_trajectory.csv "
            "build/phase12_speed_1p55_optimized_trajectory.csv"
        )
        sys.exit(1)

    summary_csv = Path(sys.argv[1])
    traj_paths = [Path(arg) for arg in sys.argv[2:]]
    summary_rows = load_csv(summary_csv)

    # 主数据流：
    # 1) 从 summary 读取每个速度工况的标签与指标；
    # 2) 从各工况轨迹文件读取 x-y；
    # 3) 左图对比轨迹，右图对比量化指标。
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    colors = ["tab:red", "tab:orange", "tab:green"]
    # zip 的隐含约束：summary 行数与轨迹文件数应对齐。
    # 如果某一侧更多，zip 会按较短长度截断，因此数据生产侧应保证一一对应。
    for color, traj_path, row in zip(colors, traj_paths, summary_rows):
      x, y = load_traj(traj_path)
      label = f"v={row['obstacle_speed']} -> {row['response']}"
      axes[0].plot(x, y, color=color, linewidth=2.2, label=label)
    axes[0].axhline(0.0, color="tab:cyan", linestyle=":", linewidth=1.2, label="reference line")
    axes[0].axhline(2.0, color="tab:olive", linestyle="--", linewidth=1.1, label="road boundary")
    axes[0].axhline(-2.0, color="tab:olive", linestyle="--", linewidth=1.1)
    axes[0].set_title("Trajectory response vs obstacle speed")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].grid(True)
    axes[0].legend(loc="upper right")

    # 指标图：观察障碍物速度提升时，延迟和横向偏移是否同步放大。
    obstacle_speed = [float(row["obstacle_speed"]) for row in summary_rows]
    crossing_delay = [float(row["crossing_delay"]) for row in summary_rows]
    max_abs_lateral = [float(row["max_abs_lateral"]) for row in summary_rows]
    axes[1].plot(obstacle_speed, crossing_delay, marker="o", color="tab:blue", label="crossing delay")
    axes[1].plot(obstacle_speed, max_abs_lateral, marker="s", color="tab:red", label="max abs lateral")
    axes[1].set_title("Speed sweep metrics")
    axes[1].set_xlabel("obstacle speed [m/s]")
    axes[1].grid(True)
    axes[1].legend()

    fig.tight_layout()
    output_path = summary_csv.with_name("phase12_speed_sweep.png")
    fig.savefig(output_path, dpi=160)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
