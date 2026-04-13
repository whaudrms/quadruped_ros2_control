#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--aggregate-csv",
        default="/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/summary/contact_timing_bias_aggregate.csv",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/summary",
    )
    parser.add_argument("--terrain-height-cm", type=float, default=5.0)
    return parser.parse_args()


def setup_style():
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "grid.linestyle": "--",
            "axes.titleweight": "semibold",
            "axes.labelsize": 11,
            "axes.titlesize": 12,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "legend.fontsize": 10,
        }
    )


def ms_formatter(x, _pos):
    return f"{int(round(x * 1000))}"


def style_axis(ax):
    ax.spines["left"].set_alpha(0.45)
    ax.spines["bottom"].set_alpha(0.45)
    ax.tick_params(length=4, width=0.8, color="#444444")
    ax.grid(True, axis="y", alpha=0.22)
    ax.grid(False, axis="x")


def load_rows(path: Path, terrain_height_cm: float):
    rows = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            if float(row["terrain_height_cm"]) != terrain_height_cm:
                continue
            rows.append(
                {
                    "d": float(row["contact_timing_bias_s"]),
                    "n_runs": int(row["n_runs"]),
                    "success_rate": float(row["success_rate"]),
                    "stable_rate": float(row["stable_rate"]),
                    "unstable_rate": float(row["unstable_rate"]),
                    "fail_rate": float(row["fail_rate"]),
                    "forward_mean": float(row["forward_progress_mean"]),
                    "forward_std": float(row["forward_progress_std"]),
                    "lateral_mean": float(row["lateral_progress_mean"]),
                    "lateral_std": float(row["lateral_progress_std"]),
                    "roll_mean": float(row["roll_rms_mean"]),
                    "roll_std": float(row["roll_rms_std"]),
                    "pitch_mean": float(row["pitch_rms_mean"]),
                    "pitch_std": float(row["pitch_rms_std"]),
                }
            )
    rows.sort(key=lambda r: r["d"])
    return rows


def save_main_figure(rows, outdir: Path, terrain_height_cm: float):
    d = [r["d"] for r in rows]
    n_runs = [r["n_runs"] for r in rows]
    forward_mean = [r["forward_mean"] for r in rows]
    forward_std = [r["forward_std"] for r in rows]
    lateral_mean = [r["lateral_mean"] for r in rows]
    lateral_std = [r["lateral_std"] for r in rows]
    roll_mean = [r["roll_mean"] for r in rows]
    roll_std = [r["roll_std"] for r in rows]
    stable_rate = [r["stable_rate"] for r in rows]
    fail_rate = [r["fail_rate"] for r in rows]

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8), constrained_layout=True)

    axes[0].fill_between(
        d,
        [m - s for m, s in zip(forward_mean, forward_std)],
        [m + s for m, s in zip(forward_mean, forward_std)],
        color="#6baed6",
        alpha=0.22,
        linewidth=0,
    )
    axes[0].plot(d, forward_mean, "-o", color="#1f77b4", lw=2.4, ms=6)
    axes[0].set_title("Forward Progress")
    axes[0].set_xlabel("Timing bias d [ms]")
    axes[0].set_ylabel("Body-frame forward progress [m]")
    axes[0].xaxis.set_major_formatter(FuncFormatter(ms_formatter))
    style_axis(axes[0])
    for x, y, n in zip(d, forward_mean, n_runs):
        axes[0].annotate(f"n={n}", (x, y), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=8, color="#444444")

    axes[1].fill_between(
        d,
        [m - s for m, s in zip(lateral_mean, lateral_std)],
        [m + s for m, s in zip(lateral_mean, lateral_std)],
        color="#fcae91",
        alpha=0.22,
        linewidth=0,
    )
    axes[1].plot(d, lateral_mean, "-o", color="#d62728", lw=2.4, ms=6)
    axes[1].axhline(0.0, color="#777777", lw=1.0, alpha=0.6)
    axes[1].set_title("Lateral Drift")
    axes[1].set_xlabel("Timing bias d [ms]")
    axes[1].set_ylabel("Body-frame lateral progress [m]")
    axes[1].xaxis.set_major_formatter(FuncFormatter(ms_formatter))
    style_axis(axes[1])

    axes[2].fill_between(
        d,
        [m - s for m, s in zip(roll_mean, roll_std)],
        [m + s for m, s in zip(roll_mean, roll_std)],
        color="#a1d99b",
        alpha=0.22,
        linewidth=0,
    )
    axes[2].plot(d, roll_mean, "-o", color="#2ca02c", lw=2.4, ms=6, label="Roll RMS")
    axes[2].plot(d, [100 * r for r in stable_rate], "--", color="#4daf4a", lw=1.8, alpha=0.85, label="Stable rate [%]")
    axes[2].plot(d, [100 * r for r in fail_rate], "--", color="#e41a1c", lw=1.8, alpha=0.85, label="Fail rate [%]")
    axes[2].set_title("Roll RMS and Outcome Rates")
    axes[2].set_xlabel("Timing bias d [ms]")
    axes[2].set_ylabel("Roll RMS [deg]")
    axes[2].xaxis.set_major_formatter(FuncFormatter(ms_formatter))
    style_axis(axes[2])
    ax2b = axes[2].twinx()
    ax2b.set_ylabel("Rate [%]")
    ax2b.set_ylim(0, 100)
    ax2b.spines["top"].set_visible(False)
    ax2b.spines["left"].set_visible(False)
    ax2b.spines["right"].set_alpha(0.45)
    ax2b.tick_params(length=4, width=0.8, color="#444444")
    ax2b.plot(d, [100 * r for r in stable_rate], alpha=0)
    ax2b.plot(d, [100 * r for r in fail_rate], alpha=0)
    handles, labels = axes[2].get_legend_handles_labels()
    axes[2].legend(handles, labels, frameon=False, loc="upper left")

    fig.suptitle(f"Pure OCS2 Under Contact-Timing Bias on a {terrain_height_cm:.0f} cm Step-Down", fontsize=14, fontweight="semibold")
    png_path = outdir / f"contact_timing_bias_main_{int(terrain_height_cm)}cm.png"
    pdf_path = outdir / f"contact_timing_bias_main_{int(terrain_height_cm)}cm.pdf"
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    return png_path, pdf_path


def save_outcome_figure(rows, outdir: Path, terrain_height_cm: float):
    d = [r["d"] for r in rows]
    n_runs = [r["n_runs"] for r in rows]
    stable = [r["stable_rate"] for r in rows]
    unstable = [r["unstable_rate"] for r in rows]
    fail = [r["fail_rate"] for r in rows]
    success = [r["success_rate"] for r in rows]

    x = list(range(len(d)))
    labels = [f"{int(round(v * 1000))}" for v in d]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)

    axes[0].bar(x, stable, color="#4daf4a", label="Stable", width=0.72)
    axes[0].bar(x, unstable, bottom=stable, color="#ffb000", label="Unstable", width=0.72)
    axes[0].bar(x, fail, bottom=[a + b for a, b in zip(stable, unstable)], color="#e41a1c", label="Fail", width=0.72)
    axes[0].set_title("Outcome Composition")
    axes[0].set_xlabel("Timing bias d [ms]")
    axes[0].set_ylabel("Rate")
    axes[0].set_ylim(0.0, 1.0)
    axes[0].set_xticks(x, labels)
    axes[0].legend(frameon=False, loc="upper right")
    style_axis(axes[0])
    for xi, n in zip(x, n_runs):
        axes[0].annotate(f"n={n}", (xi, 1.0), textcoords="offset points", xytext=(0, 4), ha="center", fontsize=8, color="#444444")

    axes[1].plot(x, success, "-o", color="#1f77b4", lw=2.4, ms=6, label="Success rate")
    axes[1].plot(x, stable, "-o", color="#4daf4a", lw=2.2, ms=5, label="Stable rate")
    axes[1].plot(x, fail, "-o", color="#e41a1c", lw=2.2, ms=5, label="Fail rate")
    axes[1].set_title("Success and Failure Rates")
    axes[1].set_xlabel("Timing bias d [ms]")
    axes[1].set_ylabel("Success rate")
    axes[1].set_ylim(0.0, 1.05)
    axes[1].set_xticks(x, labels)
    axes[1].legend(frameon=False, loc="upper right")
    style_axis(axes[1])

    fig.suptitle(f"Outcome Summary on a {terrain_height_cm:.0f} cm Step-Down", fontsize=14, fontweight="semibold")
    png_path = outdir / f"contact_timing_bias_outcomes_{int(terrain_height_cm)}cm.png"
    pdf_path = outdir / f"contact_timing_bias_outcomes_{int(terrain_height_cm)}cm.pdf"
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    return png_path, pdf_path


def main():
    args = parse_args()
    setup_style()
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    rows = load_rows(Path(args.aggregate_csv), args.terrain_height_cm)
    if not rows:
        raise SystemExit("No rows found for the requested terrain height.")
    main_png, main_pdf = save_main_figure(rows, outdir, args.terrain_height_cm)
    out_png, out_pdf = save_outcome_figure(rows, outdir, args.terrain_height_cm)
    print(f"[done] {main_png}")
    print(f"[done] {main_pdf}")
    print(f"[done] {out_png}")
    print(f"[done] {out_pdf}")


if __name__ == "__main__":
    main()
