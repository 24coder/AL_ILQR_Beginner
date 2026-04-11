#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import pandas as pd


def _finite_minmax(values: list[pd.Series]) -> tuple[float, float]:
    min_v = float("inf")
    max_v = float("-inf")
    for series in values:
        smin = float(series.min())
        smax = float(series.max())
        min_v = min(min_v, smin)
        max_v = max(max_v, smax)
    if not (min_v < float("inf") and max_v > float("-inf")):
        return -1.0, 1.0
    if abs(max_v - min_v) < 1e-9:
        return min_v - 1.0, max_v + 1.0
    return min_v, max_v


def build_animation(
    evolution_csv: Path,
    meta_csv: Path,
    obstacle_csv: Path,
    output_gif: Path,
    fps: int,
) -> None:
    evolution = pd.read_csv(evolution_csv)
    meta = pd.read_csv(meta_csv)
    obstacle = pd.read_csv(obstacle_csv)

    iteration_ids: list[int] = []
    for column in evolution.columns:
        if column.startswith("outer_") and column.endswith("_x"):
            iteration_ids.append(int(column[len("outer_") : -len("_x")]))
    iteration_ids.sort()

    if not iteration_ids:
        raise ValueError("No outer iteration trajectory columns found in trajectory evolution CSV.")

    meta = meta.set_index("outer_iteration")

    x_series = [evolution[f"outer_{idx}_x"] for idx in iteration_ids]
    y_series = [evolution[f"outer_{idx}_y"] for idx in iteration_ids]
    all_x = x_series + [obstacle["cx"]]
    all_y = y_series + [obstacle["cy"]]
    min_x, max_x = _finite_minmax(all_x)
    min_y, max_y = _finite_minmax(all_y)

    margin_x = max(1.0, 0.08 * (max_x - min_x))
    margin_y = max(1.0, 0.12 * (max_y - min_y))

    output_gif.parent.mkdir(parents=True, exist_ok=True)
    frame_paths: list[Path] = []
    temp_dir = output_gif.parent / ".phase11_iteration_frames"
    temp_dir.mkdir(parents=True, exist_ok=True)

    try:
        for frame_idx, iteration in enumerate(iteration_ids):
            if iteration not in meta.index:
                raise KeyError(f"Iteration {iteration} missing in metadata CSV.")

            row = meta.loc[iteration]
            traj_x = evolution[f"outer_{iteration}_x"]
            traj_y = evolution[f"outer_{iteration}_y"]

            fig, ax = plt.subplots(figsize=(8.5, 5.5), dpi=140)

            ax.plot(obstacle["cx"], obstacle["cy"], "--", color="0.75", lw=1.5, label="Obstacle path")
            radius = float(obstacle["radius"].iloc[0]) if not obstacle.empty else 0.0
            for _, obs_row in obstacle.iterrows():
                circle = Circle(
                    (obs_row["cx"], obs_row["cy"]),
                    radius,
                    facecolor=(0.85, 0.35, 0.35, 0.08),
                    edgecolor=(0.75, 0.25, 0.25, 0.18),
                    lw=0.8,
                )
                ax.add_patch(circle)

            ax.plot(traj_x, traj_y, color="#1f77b4", lw=2.5, label=f"Outer iter {iteration}")
            ax.scatter(traj_x.iloc[0], traj_y.iloc[0], color="#2ca02c", s=42, zorder=3, label="Start")
            ax.scatter(traj_x.iloc[-1], traj_y.iloc[-1], color="#d62728", s=42, zorder=3, label="End")

            title = (
                f"Phase11 trajectory evolution | outer={iteration} | "
                f"base_cost={float(row['base_cost']):.3f} | "
                f"violation={float(row['max_violation']):.6f}"
            )
            subtitle = (
                f"inner_iters={int(row['inner_iterations'])}, "
                f"aug_cost={float(row['augmented_cost']):.3f}, "
                f"best_violation={float(row['best_violation_so_far']):.6f}, "
                f"max_penalty={float(row['max_penalty']):.3f}, "
                f"penalty_updated={int(row['penalty_updated'])}"
            )
            ax.set_title(title, fontsize=12)
            ax.text(
                0.5,
                1.01,
                subtitle,
                transform=ax.transAxes,
                ha="center",
                va="bottom",
                fontsize=9,
                color="0.25",
            )

            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlim(min_x - margin_x, max_x + margin_x)
            ax.set_ylim(min_y - margin_y, max_y + margin_y)
            ax.grid(True, alpha=0.25)
            ax.legend(loc="best")
            fig.tight_layout()

            frame_path = temp_dir / f"frame_{frame_idx:03d}.png"
            fig.savefig(frame_path)
            plt.close(fig)
            frame_paths.append(frame_path)

        images = [imageio.imread(frame_path) for frame_path in frame_paths]
        imageio.mimsave(output_gif, images, fps=fps)
    finally:
        for frame_path in frame_paths:
            if frame_path.exists():
                frame_path.unlink()
        if temp_dir.exists():
            try:
                temp_dir.rmdir()
            except OSError:
                pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a GIF showing phase11 trajectory evolution across AL-iLQR outer iterations."
    )
    parser.add_argument(
        "--evolution-csv",
        type=Path,
        default=Path("build/phase11_dynamic_trajectory_evolution.csv"),
        help="Path to trajectory evolution CSV.",
    )
    parser.add_argument(
        "--meta-csv",
        type=Path,
        default=Path("build/phase11_dynamic_trajectory_evolution_meta.csv"),
        help="Path to trajectory evolution metadata CSV.",
    )
    parser.add_argument(
        "--obstacle-csv",
        type=Path,
        default=Path("build/phase11_dynamic_obstacle_prediction.csv"),
        help="Path to obstacle prediction CSV.",
    )
    parser.add_argument(
        "--output-gif",
        type=Path,
        default=Path("build/phase11_outer_iterations.gif"),
        help="Path to output GIF.",
    )
    parser.add_argument("--fps", type=int, default=2, help="Animation frames per second.")
    args = parser.parse_args()

    build_animation(
        evolution_csv=args.evolution_csv,
        meta_csv=args.meta_csv,
        obstacle_csv=args.obstacle_csv,
        output_gif=args.output_gif,
        fps=args.fps,
    )
    print(f"Saved GIF to {args.output_gif}")


if __name__ == "__main__":
    main()
