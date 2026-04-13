#!/usr/bin/env python3
# Usage:
#   python3 /home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/scripts/plot_contact_timing_handover_figures.py
#
# Reads:
#   /home/ho/ros2_ws/filtered_results/main_table.csv
#   /home/ho/ros2_ws/filtered_results/exclusion_overview.csv
#
# Writes:
#   /home/ho/ros2_ws/filtered_results/figures/

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


MAIN_TABLE = Path("/home/ho/ros2_ws/filtered_results/main_table.csv")
EXCLUSION_OVERVIEW = Path("/home/ho/ros2_ws/filtered_results/exclusion_overview.csv")
OUT_DIR = Path("/home/ho/ros2_ws/filtered_results/figures")
LOG_PATH = OUT_DIR / "figure_generation_log.txt"


def load_main_table():
    return pd.read_csv(MAIN_TABLE)


def preprocess_main_table(df):
    df = df.copy()
    numeric_cols = [
        "terrain_height_cm",
        "contact_timing_bias_s",
        "n_total_runs",
        "n_pass_runs",
        "n_excluded_runs",
        "success_rate",
        "stable_rate",
        "fail_rate",
        "forward_progress_mean",
        "roll_rms_deg_mean",
        "pitch_rms_deg_mean",
    ]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.sort_values(["terrain_height_cm", "contact_timing_bias_s"]).reset_index(drop=True)
    return df


def save_figure(fig, base_name, generated_files):
    png_path = OUT_DIR / f"{base_name}.png"
    pdf_path = OUT_DIR / f"{base_name}.pdf"
    fig.savefig(png_path, dpi=300, bbox_inches="tight")
    fig.savefig(pdf_path, dpi=300, bbox_inches="tight")
    generated_files.extend([str(png_path), str(pdf_path)])
    plt.close(fig)


def plot_line_metric(df, metric_col, y_label, title, base_name, generated_files, counters):
    plot_df = df[df["n_pass_runs"] > 0].copy()
    excluded_rows = int((df["n_pass_runs"].fillna(0) <= 0).sum())
    nan_rows = int(plot_df[metric_col].isna().sum())
    plot_df = plot_df.dropna(subset=[metric_col, "terrain_height_cm", "contact_timing_bias_s"])

    fig, ax = plt.subplots(figsize=(8, 5))
    for terrain in sorted(plot_df["terrain_height_cm"].dropna().unique()):
        sub = plot_df[plot_df["terrain_height_cm"] == terrain]
        ax.plot(
            sub["contact_timing_bias_s"],
            sub[metric_col],
            marker="o",
            linewidth=1.8,
            markersize=5,
            label=f"{int(terrain)} cm",
        )

    ax.set_xlabel("Contact Timing Bias [s]")
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.grid(True)
    ax.legend(title="Terrain Height")
    save_figure(fig, base_name, generated_files)

    counters["excluded_rows"] += excluded_rows
    counters["nan_rows"] += nan_rows


def plot_heatmap_metric(df, metric_col, title, base_name, generated_files, counters):
    heat_df = df[["terrain_height_cm", "contact_timing_bias_s", metric_col]].copy()
    heat_df = heat_df.sort_values(["terrain_height_cm", "contact_timing_bias_s"])
    nan_rows = int(heat_df[metric_col].isna().sum())

    pivot = heat_df.pivot(index="terrain_height_cm", columns="contact_timing_bias_s", values=metric_col)
    pivot = pivot.sort_index().sort_index(axis=1)

    fig, ax = plt.subplots(figsize=(8, 5))
    arr = pivot.to_numpy(dtype=float)
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad(color="white")
    masked = np.ma.masked_invalid(arr)
    im = ax.imshow(masked, aspect="auto", origin="lower", cmap=cmap)

    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([f"{x:.2f}" for x in pivot.columns], rotation=45, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{int(y)}" if float(y).is_integer() else f"{y:.1f}" for y in pivot.index])
    ax.set_xlabel("Contact Timing Bias [s]")
    ax.set_ylabel("Terrain Height [cm]")
    ax.set_title(title)
    ax.grid(False)
    fig.colorbar(im, ax=ax)

    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            if not np.isnan(arr[i, j]):
                ax.text(j, i, f"{arr[i, j]:.2f}", ha="center", va="center", fontsize=8, color="white")

    save_figure(fig, base_name, generated_files)
    counters["nan_rows"] += nan_rows


def write_generation_log(generated_files, counters):
    lines = [
        f"generated_figure_files: {len(generated_files)}",
        f"generated_logical_figures: {len(generated_files) // 2}",
        f"excluded_rows_for_line_plots: {counters['excluded_rows']}",
        f"nan_rows_encountered: {counters['nan_rows']}",
        "",
        "saved_files:",
    ]
    lines.extend(generated_files)
    LOG_PATH.write_text("\n".join(lines) + "\n")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    _ = pd.read_csv(EXCLUSION_OVERVIEW)
    df = preprocess_main_table(load_main_table())

    generated_files = []
    counters = {"excluded_rows": 0, "nan_rows": 0}

    plot_line_metric(
        df, "success_rate", "Success Rate", "Success Rate vs Contact Timing Bias",
        "success_rate_vs_timing_bias", generated_files, counters
    )
    plot_line_metric(
        df, "stable_rate", "Stable Rate", "Stable Rate vs Contact Timing Bias",
        "stable_rate_vs_timing_bias", generated_files, counters
    )
    plot_line_metric(
        df, "fail_rate", "Fail Rate", "Fail Rate vs Contact Timing Bias",
        "fail_rate_vs_timing_bias", generated_files, counters
    )
    plot_line_metric(
        df, "forward_progress_mean", "Forward Progress Mean [m]", "Forward Progress vs Contact Timing Bias",
        "forward_progress_vs_timing_bias", generated_files, counters
    )
    plot_line_metric(
        df, "roll_rms_deg_mean", "Roll RMS Mean [deg]", "Roll RMS vs Contact Timing Bias",
        "roll_rms_vs_timing_bias", generated_files, counters
    )
    plot_line_metric(
        df, "pitch_rms_deg_mean", "Pitch RMS Mean [deg]", "Pitch RMS vs Contact Timing Bias",
        "pitch_rms_vs_timing_bias", generated_files, counters
    )

    plot_heatmap_metric(
        df, "success_rate", "Success Rate Heatmap",
        "success_rate_heatmap", generated_files, counters
    )
    plot_heatmap_metric(
        df, "stable_rate", "Stable Rate Heatmap",
        "stable_rate_heatmap", generated_files, counters
    )
    plot_heatmap_metric(
        df, "fail_rate", "Fail Rate Heatmap",
        "fail_rate_heatmap", generated_files, counters
    )

    write_generation_log(generated_files, counters)
    print(f"Wrote figures to {OUT_DIR}")
    print(f"Wrote log to {LOG_PATH}")


if __name__ == "__main__":
    main()
