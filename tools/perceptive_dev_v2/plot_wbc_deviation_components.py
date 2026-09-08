#!/usr/bin/env python3
"""Plot mean per-component WBC deviations for the configured trial cohorts."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from paper_plot_style import (
    COMPONENT_COLORS,
    DESCENT_COMPLETE_COLOR,
    DESCENT_START_COLOR,
    SAVE_DPI,
    apply_paper_style,
    paper_figsize,
    paper_legend,
    style_paper_axis,
)
from plot_wbc_tracking_error import load_trial_wbc_analysis, select_max_contrast_successful_pair
from terrain_descent_events import descent_times, load_or_detect_events
from wbc_plot_utils import (
    command_window_in_tick_time,
    discover_trials,
    group_trials,
    offset_file_token,
    offset_label,
)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RESULTS = SCRIPT_DIR / "results"
FIG_SIZE = paper_figsize(2, 2)


apply_paper_style()


def signed_deviations(opt_x, meas_rbd):
    """Return measured-minus-planned base position and wrapped ZYX orientation."""
    position = meas_rbd[:, 3:6] - opt_x[:, 6:9]
    orientation_raw = meas_rbd[:, 0:3] - opt_x[:, 9:12]
    orientation = np.arctan2(np.sin(orientation_raw), np.cos(orientation_raw))
    return position, orientation


def load_trial_deviations(trial: dict):
    analysis = load_trial_wbc_analysis(trial)
    if analysis is None:
        return None
    return (
        analysis["time"],
        analysis["start"],
        analysis["end"],
        analysis["position"],
        analysis["orientation"],
    )


def aggregate_deviations(trials: list[dict]):
    loaded = []
    for trial in trials:
        record = load_trial_deviations(trial)
        if record is not None:
            loaded.append((trial, record))
    if not loaded:
        return None

    duration = max(end - start for _trial, (_time, start, end, _pos, _ori) in loaded)
    grid = np.arange(0.0, duration + 1e-9, 0.005)
    position_stack, orientation_stack = [], []
    for _trial, (time, start, end, position, orientation) in loaded:
        query = grid + start
        valid = (query >= time[0]) & (query <= min(time[-1], end))
        position_values = np.full((grid.size, 3), np.nan)
        orientation_values = np.full((grid.size, 3), np.nan)
        for component in range(3):
            position_values[valid, component] = np.interp(
                query[valid], time, position[:, component]
            )
            orientation_values[valid, component] = np.interp(
                query[valid], time, orientation[:, component]
            )
        position_stack.append(position_values)
        orientation_stack.append(orientation_values)

    position_stack = np.asarray(position_stack)
    orientation_stack = np.asarray(orientation_stack)
    output = {
        "time": grid,
        "duration": duration,
        "n": len(loaded),
        "sample_count": np.sum(np.isfinite(position_stack[:, :, 0]), axis=0),
        "position_mean": np.nanmean(position_stack, axis=0),
        "position_std": np.nanstd(position_stack, axis=0),
        "orientation_mean": np.nanmean(orientation_stack, axis=0),
        "orientation_std": np.nanstd(orientation_stack, axis=0),
    }

    starts, completes = [], []
    for trial, (_time, command_start, _command_end, _position, _orientation) in loaded:
        event_start, event_complete = descent_times(load_or_detect_events(trial["trial_dir"]))
        if event_start is not None:
            display = event_start - command_start
            if 0.0 <= display <= duration:
                starts.append(display)
        if event_complete is not None:
            display = event_complete - command_start
            if 0.0 <= display <= duration:
                completes.append(display)
    output.update(
        {
            "descent_start_mean": float(np.mean(starts)) if starts else None,
            "descent_start_std": float(np.std(starts)) if starts else None,
            "descent_start_n": len(starts),
            "descent_complete_mean": float(np.mean(completes)) if completes else None,
            "descent_complete_n": len(completes),
        }
    )
    return output


def add_descent_markers(axis, data: dict):
    start = data.get("descent_start_mean")
    complete = data.get("descent_complete_mean")
    if start is not None:
        axis.axvline(
            start,
            color=DESCENT_START_COLOR,
            linestyle="--",
            linewidth=1.2,
            label=None,
        )
    if complete is not None:
        axis.axvline(
            complete,
            color=DESCENT_COMPLETE_COLOR,
            linestyle=":",
            linewidth=1.2,
            label=None,
        )
def shared_symmetric_limit(
    condition_data: tuple[dict | None, ...], mean_key: str, std_key: str
) -> float:
    """Return a common ±y limit that contains both conditions and their std bands."""
    maxima = []
    for data in condition_data:
        if data is None:
            continue
        mean = np.asarray(data[mean_key])
        std = np.asarray(data[std_key])
        extent = np.abs(mean) + std
        finite = extent[np.isfinite(extent)]
        if finite.size:
            maxima.append(float(np.max(finite)))
    largest = max(maxima, default=1.0)
    return max(largest * 1.05, 1e-9)


def plot_one_offset(offset: float, on_data: dict | None, off_data: dict | None, cohort: str, output: Path):
    fig, axes = plt.subplots(2, 2, figsize=FIG_SIZE, sharex=True)
    component_colors = COMPONENT_COLORS
    position_names = ("x", "y", "z")
    orientation_names = ("yaw", "pitch", "roll")
    conditions = (
        (0, on_data, "Proposed"),
        (1, off_data, "Baseline"),
    )
    durations = []
    selected_pair = cohort == "max_contrast_successful_pair"
    series_summary_label = "selected-trial" if selected_pair else "mean"
    row_limits = (
        shared_symmetric_limit((on_data, off_data), "position_mean", "position_std"),
        shared_symmetric_limit((on_data, off_data), "orientation_mean", "orientation_std"),
    )

    for column, data, condition_label in conditions:
        if data is None:
            for row in range(2):
                axes[row, column].set_title(f"{condition_label} — no usable trials")
            continue
        durations.append(data["duration"])
        time = data["time"]
        panels = (
            (0, data["position_mean"], data["position_std"], position_names, "position"),
            (1, data["orientation_mean"], data["orientation_std"], orientation_names, "orientation"),
        )
        for row, mean, std, names, quantity in panels:
            axis = axes[row, column]
            for component, (name, color) in enumerate(zip(names, component_colors)):
                axis.plot(time, mean[:, component], color=color, linewidth=2.25, label=name)
                axis.fill_between(
                    time,
                    mean[:, component] - std[:, component],
                    mean[:, component] + std[:, component],
                    color=color,
                    alpha=0.10,
                    linewidth=0,
                )
            add_descent_markers(axis, data)
            axis.set_title(
                f"{condition_label}: {series_summary_label} {quantity} deviation",
            )
            if row == 1:
                axis.set_xlabel("Command-active time [s]")

    display_duration = max(durations) if durations else 1.0
    for row in range(2):
        for column in range(2):
            axis = axes[row, column]
            axis.set_xlim(0, display_duration)
            axis.set_ylim(-row_limits[row], row_limits[row])
            style_paper_axis(axis)
            axis.axhline(0, color="black", linewidth=0.5, alpha=0.5)
    axes[0, 0].set_ylabel("Position error [m]")
    axes[1, 0].set_ylabel("Orientation error [rad]")
    cohort_title = {
        "all_trials": "all trials",
        "successful_only": "successful trials only",
        "max_contrast_successful_pair": "max-contrast successful pair (selected)",
    }[cohort]
    fig.suptitle(
        f"{'Selected' if selected_pair else 'Mean'} WBC Tracking Deviation - "
        f"{offset_label(offset)} - {cohort_title.title()}",
        y=0.995,
    )
    legend_handles = [
        Line2D([], [], color=color, linewidth=2.25, label=name)
        for name, color in zip(("x / yaw", "y / pitch", "z / roll"), component_colors)
    ]
    legend_handles.extend(
        (
            Line2D([], [], color=DESCENT_START_COLOR, linestyle="--", label="First lower touchdown"),
            Line2D([], [], color=DESCENT_COMPLETE_COLOR, linestyle=":", label="Descent complete"),
        )
    )
    paper_legend(
        fig, handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
        ncol=3,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=SAVE_DPI)
    plt.close(fig)
    print(f"saved {output}")


def select_cohort(trials: list[dict], cohort: str):
    if cohort == "successful_only":
        return [trial for trial in trials if bool(trial.get("result", {}).get("success"))]
    if cohort == "max_contrast_successful_pair":
        return select_max_contrast_successful_pair(trials)[0]
    return trials


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument(
        "--cohort",
        choices=("all", "all_trials", "successful_only", "max_contrast_successful_pair"),
        default="all",
        help="Generate all three comparison versions by default.",
    )
    args = parser.parse_args(argv)

    results_dir = args.results_dir.resolve()
    out_dir = (args.out_dir or results_dir / "all_visualizations").resolve()
    all_trials = discover_trials(results_dir)
    if not all_trials:
        raise RuntimeError(f"No WBC-compatible trials found in {results_dir}")
    cohorts = (
        ("all_trials", "successful_only", "max_contrast_successful_pair")
        if args.cohort == "all"
        else (args.cohort,)
    )
    for cohort in cohorts:
        groups = group_trials(select_cohort(all_trials, cohort))
        offsets = sorted({offset for offset, _robust in groups})
        for offset in offsets:
            on_data = aggregate_deviations(groups.get((offset, "ON"), []))
            off_data = aggregate_deviations(groups.get((offset, "OFF"), []))
            output = out_dir / (
                f"wbc_deviation_components_{cohort}_v2_dz{offset_file_token(offset)}.png"
            )
            plot_one_offset(offset, on_data, off_data, cohort, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
