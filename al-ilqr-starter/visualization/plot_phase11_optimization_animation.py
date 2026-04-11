#!/usr/bin/env python3
"""
动态演示 AL-iLQR 优化轨迹的收敛过程。

画面左侧：轨迹逐轮演化动画（每帧 = 一次 AL 外层迭代后的轨迹）
画面右侧：内层 iLQR 代价收敛曲线（每帧高亮当前外层迭代的收敛过程）

用法：
  python3 visualization/plot_phase11_optimization_animation.py
  python3 visualization/plot_phase11_optimization_animation.py --build-dir build \
      --output-gif build/phase11_optimization_animation.gif --no-show

依赖：
  <build-dir>/phase11_dynamic_initial_trajectory.csv
  <build-dir>/phase11_dynamic_trajectory_evolution.csv
  <build-dir>/phase11_dynamic_obstacle_prediction.csv
  <build-dir>/phase11_dynamic_outer_log.csv
  <build-dir>/phase11_dynamic_inner_cost_history.csv
"""

import argparse
import csv
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle


# ── 数据加载 ──────────────────────────────────────────────────────────────────

def load_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_trajectory_evolution(path):
    """返回 {outer_idx: (xs, ys)} 字典，outer_idx 从 1 开始。"""
    rows = load_csv(path)
    headers = list(rows[0].keys())  # k, outer_1_x, outer_1_y, outer_2_x, ...
    outer_cols = {}
    for h in headers:
        if h.startswith("outer_") and h.endswith("_x"):
            idx = int(h.split("_")[1])
            outer_cols[idx] = h[:-2]  # strip "_x"

    trajs = {}
    for idx in sorted(outer_cols):
        prefix = outer_cols[idx]
        xs = [float(r[prefix + "_x"]) for r in rows]
        ys = [float(r[prefix + "_y"]) for r in rows]
        trajs[idx] = (xs, ys)
    return trajs


def load_inner_cost_history(path):
    """返回 {outer_iter: [costs]} 字典。"""
    rows = load_csv(path)
    history = {}
    for r in rows:
        oi = int(r["outer_iter"])
        c = float(r["augmented_cost"])
        history.setdefault(oi, []).append(c)
    return history


def load_outer_log(path):
    rows = load_csv(path)
    return [
        {
            "iter": int(r["outer_iteration"]),
            "inner_iters": int(r["inner_iterations"]),
            "base_cost": float(r["base_cost"]),
            "violation": float(r["max_violation"]),
            "penalty": float(r["max_penalty"]),
            "penalty_updated": r["penalty_updated"] == "1",
        }
        for r in rows
    ]


def build_animation(build_dir: Path, output_gif: Path, show: bool) -> None:
    # 数据文件路径
    init_csv = build_dir / "phase11_dynamic_initial_trajectory.csv"
    evolution_csv = build_dir / "phase11_dynamic_trajectory_evolution.csv"
    obstacle_csv = build_dir / "phase11_dynamic_obstacle_prediction.csv"
    outer_log_csv = build_dir / "phase11_dynamic_outer_log.csv"
    inner_cost_csv = build_dir / "phase11_dynamic_inner_cost_history.csv"

    # 加载数据
    init_rows = load_csv(init_csv)
    x_init = [float(r["x0"]) for r in init_rows]
    y_init = [float(r["x1"]) for r in init_rows]

    trajs = load_trajectory_evolution(evolution_csv)
    outer_iters = sorted(trajs.keys())
    n_frames = len(outer_iters)

    obs_rows = load_csv(obstacle_csv)
    obs_x = [float(r["cx"]) for r in obs_rows]
    obs_y = [float(r["cy"]) for r in obs_rows]
    obs_r = float(obs_rows[0]["radius"])

    outer_log = load_outer_log(outer_log_csv)
    inner_hist = load_inner_cost_history(inner_cost_csv)

    road_half = 2.3

    # ── 图层布局 ──────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(15, 7), facecolor="#0f1117")
    fig.patch.set_facecolor("#0f1117")

    gs = gridspec.GridSpec(
        2, 2,
        figure=fig,
        left=0.07, right=0.97,
        top=0.90, bottom=0.10,
        wspace=0.35, hspace=0.45,
    )

    ax_traj = fig.add_subplot(gs[:, 0])   # 左侧大图：轨迹演化
    ax_cost = fig.add_subplot(gs[0, 1])   # 右上：内层 iLQR cost 曲线
    ax_info = fig.add_subplot(gs[1, 1])   # 右下：外层指标信息

    for ax in [ax_traj, ax_cost, ax_info]:
        ax.set_facecolor("#1a1d27")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444")

    COLORS = {
        "init": "#555577",
        "past": "#3a5a7a",
        "current": "#00d4ff",
        "obstacle": "#ff4444",
        "road": "#f5c518",
        "ref": "#44cc88",
        "cost_bg": "#4a6080",
        "cost_hi": "#ffcc00",
    }

    # ── 轨迹面板初始化 ────────────────────────────────────────────────────────
    ax_traj.set_title("Trajectory Evolution (AL-iLQR Outer Iterations)",
                      color="white", fontsize=12, pad=8)
    ax_traj.set_xlabel("x [m]", color="#aaa")
    ax_traj.set_ylabel("y [m]", color="#aaa")
    ax_traj.tick_params(colors="#888")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.grid(True, color="#333", linewidth=0.5)

    # 静态背景：道路边界、参考线
    ax_traj.axhline(road_half, color=COLORS["road"], linestyle="--",
                    linewidth=1.2, alpha=0.7, label="road boundary")
    ax_traj.axhline(-road_half, color=COLORS["road"], linestyle="--", linewidth=1.2, alpha=0.7)
    ax_traj.axhspan(-road_half, road_half, color="#ffee0010", linewidth=0)
    ax_traj.axhline(0.0, color=COLORS["ref"], linestyle=":", linewidth=1.2, label="reference line")

    # 初始猜测轨迹（贯穿全程，灰色虚线）
    ax_traj.plot(x_init, y_init, "--", color=COLORS["init"],
                 linewidth=1.5, alpha=0.6, label="initial guess", zorder=2)

    # 障碍物预测路径（细红线）
    ax_traj.plot(obs_x, obs_y, color=COLORS["obstacle"],
                 linewidth=1.0, alpha=0.5, zorder=2)

    # 用不同时刻的障碍圆表示预测轨迹（抽样）
    sample_ids = np.linspace(0, len(obs_rows) - 1, 7, dtype=int)
    for sid in sample_ids:
        alpha = 0.08 + 0.12 * sid / max(1, len(obs_rows) - 1)
        ax_traj.add_patch(Circle(
            (obs_x[sid], obs_y[sid]), obs_r,
            color=COLORS["obstacle"], alpha=alpha, zorder=2
        ))

    # 动态元素：历史轨迹（越旧越暗）+ 当前轨迹
    past_lines = []
    MAX_GHOST = 4  # 保留最近几条历史轨迹作为"幽灵"
    for _ in range(MAX_GHOST):
        ln, = ax_traj.plot([], [], "-", color=COLORS["past"],
                           linewidth=1.0, alpha=0.0, zorder=3)
        past_lines.append(ln)

    curr_line, = ax_traj.plot([], [], "-", color=COLORS["current"],
                              linewidth=2.5, zorder=5, label="current iter")
    curr_dot, = ax_traj.plot([], [], "o", color=COLORS["current"],
                             markersize=6, zorder=6)

    # 标题文字
    iter_text = ax_traj.text(
        0.02, 0.97, "", transform=ax_traj.transAxes,
        color="white", fontsize=11, va="top",
        bbox=dict(boxstyle="round,pad=0.3", fc="#00000080", ec="none")
    )

    # 图例
    legend_handles = [
        Line2D([0], [0], color=COLORS["init"], linestyle="--", label="initial guess"),
        Line2D([0], [0], color=COLORS["current"], linestyle="-", label="current iter"),
        Line2D([0], [0], color=COLORS["past"], linestyle="-", label="past iters"),
        Line2D([0], [0], color=COLORS["ref"], linestyle=":", label="reference"),
        Line2D([0], [0], color=COLORS["road"], linestyle="--", label="road boundary"),
        Line2D([0], [0], color=COLORS["obstacle"], linestyle="-", label="obstacle path"),
    ]
    ax_traj.legend(handles=legend_handles, loc="upper right",
                   facecolor="#1a1d27", edgecolor="#555", labelcolor="white", fontsize=8)

    # 设定 x/y 范围
    all_x = x_init + [x for xs, _ in trajs.values() for x in xs]
    all_y = y_init + [y for _, ys in trajs.values() for y in ys]
    xm, xM = min(all_x) - 0.5, max(all_x) + 0.5
    ym, yM = min(all_y + [-road_half]) - 0.3, max(all_y + [road_half]) + 0.3
    ax_traj.set_xlim(xm, xM)
    ax_traj.set_ylim(ym, yM)

    # ── 代价收敛面板初始化 ───────────────────────────────────────────────────
    ax_cost.set_title("Inner iLQR Cost Convergence", color="white", fontsize=10, pad=6)
    ax_cost.set_xlabel("inner iteration", color="#aaa")
    ax_cost.set_ylabel("augmented cost", color="#aaa")
    ax_cost.tick_params(colors="#888")
    ax_cost.grid(True, color="#333", linewidth=0.5)

    # 预先画出所有外层迭代的代价曲线（暗色背景）
    all_costs_lines = {}
    for oi in sorted(inner_hist):
        costs = inner_hist[oi]
        steps = list(range(len(costs)))
        ln, = ax_cost.plot(steps, costs, "-", color=COLORS["cost_bg"],
                           linewidth=0.8, alpha=0.4, zorder=2)
        all_costs_lines[oi] = ln

    # 当前外层迭代高亮线
    hi_line, = ax_cost.plot([], [], "-o", color=COLORS["cost_hi"],
                            linewidth=2.0, markersize=3, zorder=5, label="current outer iter")
    ax_cost.legend(facecolor="#1a1d27", edgecolor="#555", labelcolor="white", fontsize=8)

    # ── 外层指标面板 ──────────────────────────────────────────────────────────
    ax_info.set_title("AL Outer Loop Metrics", color="white", fontsize=10, pad=6)
    ax_info.set_facecolor("#1a1d27")
    ax_info.axis("off")

    info_text = ax_info.text(
        0.05, 0.90, "", transform=ax_info.transAxes,
        color="white", fontsize=10, va="top", family="monospace",
        linespacing=1.8,
    )

    # 外层迭代指标棒图（violation 随外层迭代变化）
    ax_bar = ax_info.inset_axes([0.0, 0.0, 1.0, 0.40])
    ax_bar.set_facecolor("#1a1d27")
    ax_bar.tick_params(colors="#888", labelsize=7)
    ax_bar.set_xlabel("outer iter", color="#aaa", fontsize=8)
    ax_bar.set_ylabel("violation", color="#aaa", fontsize=8)
    ax_bar.grid(True, color="#333", linewidth=0.5)
    for spine in ax_bar.spines.values():
        spine.set_edgecolor("#444")

    viol_iters = [lg["iter"] for lg in outer_log]
    viol_vals = [lg["violation"] for lg in outer_log]
    bar_colors_all = ["#4a6080"] * len(outer_log)
    bars = ax_bar.bar(viol_iters, viol_vals, color=bar_colors_all, width=0.6, zorder=3)
    ax_bar.set_xlim(0.3, max(viol_iters) + 0.7)

    # 全局大标题
    fig.suptitle("AL-iLQR Optimization Process — Dynamic Obstacle Avoidance",
                 color="white", fontsize=13, fontweight="bold", y=0.97)

    # ── 动画更新函数 ──────────────────────────────────────────────────────────
    def update(frame_idx):
        oi = outer_iters[frame_idx]
        xs, ys = trajs[oi]

        # 1) 更新历史幽灵轨迹
        for gi, pl in enumerate(past_lines):
            hi = frame_idx - (MAX_GHOST - gi)
            if 0 <= hi < frame_idx:
                hoi = outer_iters[hi]
                hxs, hys = trajs[hoi]
                alpha = 0.12 + 0.18 * (gi / MAX_GHOST)
                pl.set_data(hxs, hys)
                pl.set_alpha(alpha)
            else:
                pl.set_data([], [])
                pl.set_alpha(0.0)

        # 2) 更新当前轨迹
        curr_line.set_data(xs, ys)
        curr_dot.set_data([xs[-1]], [ys[-1]])

        # 3) 迭代信息标注
        log_entry = outer_log[frame_idx] if frame_idx < len(outer_log) else {}
        violation = log_entry.get("violation", 0.0)
        base_cost = log_entry.get("base_cost", 0.0)
        penalty = log_entry.get("penalty", 0.0)
        penalty_updated = log_entry.get("penalty_updated", False)
        iter_text.set_text(
            f"AL outer iter {oi}/{outer_iters[-1]}\n"
            f"violation = {violation:.4f}\n"
            f"penalty   = {penalty:.0f}"
        )

        # 4) 高亮当前外层迭代的内层代价曲线
        costs = inner_hist.get(oi, [])
        if costs:
            hi_line.set_data(range(len(costs)), costs)
        else:
            hi_line.set_data([], [])
        for k, ln in all_costs_lines.items():
            ln.set_alpha(0.6 if k == oi else 0.2)

        # 5) 更新棒图颜色（高亮当前轮）
        for i, bar in enumerate(bars):
            bar.set_color("#ffcc00" if (i + 1) == oi else "#4a6080")
            bar.set_alpha(1.0 if (i + 1) == oi else 0.5)

        # 6) 更新右上角文字
        penalty_tag = " [penalty updated]" if penalty_updated else ""
        info_text.set_text(
            f"Outer iteration : {oi:>2d} / {outer_iters[-1]}\n"
            f"Inner iLQR steps: {log_entry.get('inner_iters', '?')}\n"
            f"Base cost       : {base_cost:>10.3f}\n"
            f"Max violation   : {violation:>10.6f}\n"
            f"Max penalty (mu): {penalty:>10.0f}{penalty_tag}"
        )

        return [curr_line, curr_dot, iter_text, hi_line, info_text] + past_lines + list(bars)

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=n_frames,
        interval=800,
        blit=False,
        repeat=True,
        repeat_delay=2000,
    )

    output_gif.parent.mkdir(parents=True, exist_ok=True)
    print(f"Saving animation to {output_gif} ...")
    ani.save(str(output_gif), writer="pillow", fps=1.2, dpi=130)
    print(f"Saved: {output_gif}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate phase11 optimization animation GIF.")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("build"),
        help="Directory containing phase11 CSV outputs.",
    )
    parser.add_argument(
        "--output-gif",
        type=Path,
        default=None,
        help="Path to output GIF. Defaults to <build-dir>/phase11_optimization_animation.gif.",
    )
    parser.add_argument(
        "--show",
        dest="show",
        action="store_true",
        help="Display the matplotlib window after saving the GIF.",
    )
    parser.add_argument(
        "--no-show",
        dest="show",
        action="store_false",
        help="Skip showing the matplotlib window and only save the GIF.",
    )
    parser.set_defaults(show=False)
    args = parser.parse_args()

    output_gif = args.output_gif or (args.build_dir / "phase11_optimization_animation.gif")
    build_animation(args.build_dir, output_gif, args.show)


if __name__ == "__main__":
    main()
