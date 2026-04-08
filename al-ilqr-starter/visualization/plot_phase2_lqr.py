#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


# 读取 CSV 并返回逐行字典。
# CSV 字段约定（由 phase2 轨迹导出器提供）：
# - k: 离散时域索引
# - x0: 状态第 0 维（此脚本中按位置理解）
# - x1: 状态第 1 维（此脚本中按速度理解）
# - u0: 控制输入（末端步可能为空字符串）
def load_csv(path: Path):
    rows = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def main():
    if len(sys.argv) != 2:
        print("Usage: python visualization/plot_phase2_lqr.py build/phase2_linear_lqr_trajectory.csv")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    rows = load_csv(csv_path)

    # -------------------------
    # 主数据流：CSV -> 数值数组 -> 子图绘制 -> PNG 输出
    # -------------------------
    # 将字符串列转换为可绘图的数值序列；u0 为空时记为 NaN，
    # 这样 matplotlib 会自动断开该点，避免把“无控制定义”的终端步误连线。
    k = [int(row["k"]) for row in rows]
    x0 = [float(row["x0"]) for row in rows]
    x1 = [float(row["x1"]) for row in rows]
    u0 = [float(row["u0"]) if row["u0"] else float("nan") for row in rows]

    # 绘图意图：用三个共享 x 轴的时序图，快速检查 LQR 结果是否平滑、
    # 状态收敛是否合理、控制量是否存在突变。
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(k, x0, label="position")
    axes[0].set_ylabel("x0")
    axes[0].grid(True)

    axes[1].plot(k, x1, label="velocity", color="tab:orange")
    axes[1].set_ylabel("x1")
    axes[1].grid(True)

    axes[2].plot(k, u0, label="control", color="tab:green")
    axes[2].set_ylabel("u0")
    axes[2].set_xlabel("k")
    axes[2].grid(True)

    fig.suptitle("Phase 2 Finite-Horizon LQR")
    fig.tight_layout()
    # 输出文件与输入 CSV 同名同目录，仅后缀变为 .png，便于批处理定位结果。
    output_path = csv_path.with_suffix(".png")
    fig.savefig(output_path, dpi=150)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
