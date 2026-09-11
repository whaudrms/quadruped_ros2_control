#!/usr/bin/env python3
"""Create paper Fig. 10/11 time-resolved RMSE plots from successful MC trials."""

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from paper_plot_style import apply_figure_font_sizes
from matplotlib.lines import Line2D

from paper_plot_style import (
    NOMINAL_COLOR,
    PROPOSED_COLOR,
    REFERENCE_COLOR,
    SAVE_DPI,
    apply_paper_style,
    paper_legend,
    style_paper_axis,
)
from plot_wbc_tracking_error import load_trial_errors, select_cohort
from terrain_descent_events import descent_times, load_or_detect_events
from wbc_plot_utils import discover_trials


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RESULTS = SCRIPT_DIR / "results" / "monte_carlo_n50_seed20260831"
COMPONENT_COUNT = 3.0
CONFIDENCE_Z = 1.96
TIME_STEP_S = 0.005


def _event_aligned_time_limits(trials: list[dict]):
    relative_starts = []
    relative_ends = []
    for trial in trials:
        record = load_trial_errors(trial)
        if record is None:
            continue
        _time, start, end, _errors = record
        touchdown, _descent_complete = descent_times(
            load_or_detect_events(trial["trial_dir"])
        )
        if touchdown is None:
            continue
        relative_starts.append(start - touchdown)
        relative_ends.append(end - touchdown)
    if not relative_starts:
        raise RuntimeError("No first-lower-touchdown events found for time alignment")
    lower = np.floor(min(relative_starts) * 10.0) / 10.0
    upper = np.ceil(max(relative_ends) * 10.0) / 10.0
    return float(lower), float(upper)


def _aggregate_timewise_rmse(
    trials: list[dict], metric_index: int, time_limits: tuple[float, float]
):
    """Return first-touchdown-aligned mean and 95% CI of per-trial RMSE."""
    time_grid = np.arange(time_limits[0], time_limits[1] + 1e-9, TIME_STEP_S)
    stack = []
    for trial in trials:
        record = load_trial_errors(trial)
        if record is None:
            continue
        time, start, end, errors = record
        touchdown, _descent_complete = descent_times(
            load_or_detect_events(trial["trial_dir"])
        )
        if touchdown is None:
            continue

        query = time_grid + touchdown
        squared_norm = np.full(time_grid.shape, np.nan)
        valid = (query >= time[0]) & (query <= min(time[-1], end))
        valid &= query >= start
        squared_norm[valid] = np.interp(query[valid], time, errors[metric_index])
        stack.append(np.sqrt(np.maximum(squared_norm / COMPONENT_COUNT, 0.0)))

    if not stack:
        return None

    values = np.asarray(stack, dtype=float)
    count = np.sum(np.isfinite(values), axis=0)
    mean = np.divide(
        np.nansum(values, axis=0),
        count,
        out=np.full(time_grid.shape, np.nan),
        where=count > 0,
    )
    sample_std = np.full(mean.shape, np.nan)
    for index, n_valid in enumerate(count):
        if n_valid > 1:
            sample_std[index] = np.nanstd(values[:, index], ddof=1)
        elif n_valid == 1:
            sample_std[index] = 0.0
    half_width = CONFIDENCE_Z * sample_std / np.sqrt(np.maximum(count, 1))

    return {
        "t_grid": time_grid,
        "n": len(stack),
        "sample_count": count,
        "mean": mean,
        "lower": np.maximum(mean - half_width, 0.0),
        "upper": mean + half_width,
    }


def _plot_metric(
    method_data: dict[str, dict],
    ylabel: str,
    output: Path,
    figure_number: int,
    terrain_range_m: tuple[float, float],
    time_limits: tuple[float, float],
):
    fig, axis = plt.subplots(figsize=(8.0, 4.5))
    styles = {
        "OFF": {"color": NOMINAL_COLOR, "label": "Baseline"},
        "ON": {"color": PROPOSED_COLOR, "label": "Proposed"},
    }

    report_methods = {}
    ci_ceiling = 0.0
    for robust, style in styles.items():
        data = method_data.get(robust)
        if data is None:
            continue
        time = data["t_grid"]
        axis.fill_between(
            time,
            data["lower"],
            data["upper"],
            color=style["color"],
            alpha=0.18,
            linewidth=0,
            zorder=1,
        )
        axis.plot(
            time,
            data["mean"],
            color=style["color"],
            linewidth=2.25,
            label=style["label"],
            zorder=3,
        )
        ci_ceiling = max(ci_ceiling, float(np.nanmax(data["upper"])))
        report_methods[robust] = {
            "method": style["label"],
            "successful_trial_count": int(data["n"]),
            "minimum_timewise_sample_count": int(np.nanmin(data["sample_count"])),
            "time_mean_of_mean_rmse": float(np.nanmean(data["mean"])),
            "peak_mean_rmse": float(np.nanmax(data["mean"])),
        }

    axis.axvline(
        0.0,
        color=REFERENCE_COLOR,
        linestyle="--",
        linewidth=1.1,
        alpha=0.75,
        zorder=2,
    )
    axis.set_xlabel("Time [s]")
    axis.set_ylabel(ylabel)
    axis.set_xlim(*time_limits)
    axis.set_ylim(0.0, ci_ceiling / 0.72 if ci_ceiling > 0.0 else 1.0)
    style_paper_axis(axis, minor=True)
    paper_legend(
        axis,
        handles=[
            Line2D([], [], color=NOMINAL_COLOR, linewidth=2.25, label="Baseline"),
            Line2D([], [], color=PROPOSED_COLOR, linewidth=2.25, label="Proposed"),
            Line2D(
                [], [], color=REFERENCE_COLOR, linestyle="--", linewidth=1.1,
                alpha=0.75, label="Lower step contact",
            ),
        ],
        loc="upper right",
        fontsize=11,
    )
    apply_figure_font_sizes(fig, output)
    fig.tight_layout(pad=0.35)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=SAVE_DPI)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)

    report = {
        "figure": figure_number,
        "source": "Monte Carlo experiment",
        "cohort": "successful trials only",
        "terrain_height_error_range_m": list(terrain_range_m),
        "terrain_height_error_grouping": "all sampled delta-z values pooled",
        "time_alignment": "each trial's first lower touchdown is t=0",
        "plot_time_limits_s": list(time_limits),
        "rmse_definition": (
            "For each successful trial and time, sqrt(sum of squared errors over "
            "the three base components / 3); the plotted line is the across-trial mean."
        ),
        "uncertainty_band": (
            "Across-trial 95% confidence interval of the mean at each time "
            "(mean +/- 1.96 * sample standard deviation / sqrt(n), clipped at zero)."
        ),
        "methods": report_methods,
    }
    output.with_name(f"{output.stem}_selection.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(f"saved {output}")
    return output


def plot(results_dir: Path, out_dir: Path):
    apply_paper_style()
    if not (results_dir / "samples.csv").is_file():
        raise RuntimeError(f"Not a Monte Carlo result directory: {results_dir}")

    trials = discover_trials(results_dir)
    successful = select_cohort(trials, "successful_only")
    terrain_range_m = (
        float(min(trial["offset_m"] for trial in trials)),
        float(max(trial["offset_m"] for trial in trials)),
    )
    groups = {
        robust: [trial for trial in successful if trial["robust"] == robust]
        for robust in ("OFF", "ON")
    }
    if not all(groups.values()):
        raise RuntimeError(
            f"Successful Baseline/Proposed trials were not both found in {results_dir}"
        )
    time_limits = _event_aligned_time_limits(successful)

    metric_specs = (
        (
            1,
            "Base orientation RMSE [deg]",
            "fig10_base_orientation_rmse_successful.png",
            10,
        ),
        (
            0,
            "Base position RMSE [m]",
            "fig11_base_position_rmse_successful.png",
            11,
        ),
    )
    outputs = []
    for metric_index, ylabel, filename, figure_number in metric_specs:
        aggregated = {
            robust: _aggregate_timewise_rmse(group, metric_index, time_limits)
            for robust, group in groups.items()
        }
        aggregated = {key: value for key, value in aggregated.items() if value is not None}
        outputs.append(
            _plot_metric(
                aggregated,
                ylabel,
                out_dir / filename,
                figure_number,
                terrain_range_m,
                time_limits,
            )
        )
    return outputs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--out-dir", type=Path)
    args = parser.parse_args(argv)
    results_dir = args.results_dir.resolve()
    out_dir = (args.out_dir or results_dir / "all_visualizations").resolve()
    plot(results_dir, out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
