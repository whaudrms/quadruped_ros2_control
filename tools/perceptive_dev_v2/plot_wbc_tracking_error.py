#!/usr/bin/env python3
"""Plot WBC tracking error aggregated over every condition in a results folder.

The script discovers trials from run_config.json/result.json instead of relying
on timestamped directory names. Errors compare the MPC optimized state (opt_x)
with the measured rigid-body state (meas_rbd) during each scenario's recorded
command-active window.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from paper_plot_style import apply_figure_font_sizes
from matplotlib.lines import Line2D

from paper_plot_style import (
    DESCENT_COMPLETE_COLOR,
    DESCENT_START_COLOR,
    NOMINAL_COLOR,
    PROPOSED_COLOR,
    SAVE_DPI,
    apply_paper_style,
    paper_figsize,
    paper_legend,
    style_paper_axis,
)
from wbc_plot_utils import (
    command_window_in_tick_time,
    discover_trials,
    group_trials,
    offset_label,
)
from terrain_descent_events import descent_times, load_or_detect_events


apply_paper_style()


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RESULTS = SCRIPT_DIR / "results"
WBC_CACHE_VERSION = 1


def load_tick(tick_csv: Path):
    """Return t, opt_x[24], meas_rbd[36], and planned_mode arrays."""
    try:
        stream = tick_csv.open(encoding="utf-8")
    except OSError:
        return None
    with stream:
        reader = csv.reader(stream)
        try:
            header = next(reader)
            t_col = header.index("t")
            opt_x_cols = [header.index(f"opt_x{i}") for i in range(24)]
            meas_cols = [header.index(f"meas_rbd{i}") for i in range(36)]
            mode_col = header.index("planned_mode")
        except (StopIteration, ValueError):
            return None
        n_expected = len(header)
        ts, opt_xs, meas_xs, modes = [], [], [], []
        for row in reader:
            if len(row) != n_expected:
                continue
            try:
                vals = [float(value) for value in row]
            except ValueError:
                continue
            ts.append(vals[t_col])
            opt_xs.append([vals[column] for column in opt_x_cols])
            meas_xs.append([vals[column] for column in meas_cols])
            modes.append(int(vals[mode_col]))
    if not ts:
        return None
    return np.asarray(ts), np.asarray(opt_xs), np.asarray(meas_xs), np.asarray(modes)


def compute_errors(opt_x, meas_rbd):
    """Return squared-norm position, orientation, and joint tracking errors."""
    base_pos = np.sum((meas_rbd[:, 3:6] - opt_x[:, 6:9]) ** 2, axis=1)
    orientation_deg = np.degrees(meas_rbd[:, 0:3] - opt_x[:, 9:12])
    base_orientation = np.sum(orientation_deg**2, axis=1)
    joint_position = np.sum((meas_rbd[:, 6:18] - opt_x[:, 12:24]) ** 2, axis=1)
    return base_pos, base_orientation, joint_position


def load_trial_wbc_analysis(trial: dict):
    """Load compact derived WBC series, parsing the large tick CSV only once."""
    tick_path = trial["tick_path"]
    cache_path = trial["trial_dir"] / "wbc_analysis_cache.npz"
    tick_stat = tick_path.stat()
    metadata_paths = [
        trial["trial_dir"] / "result.json",
        trial["trial_dir"] / "run_config.json",
        trial["trial_dir"] / "scenario.yaml",
    ]
    metadata_mtime_ns = max(
        (path.stat().st_mtime_ns for path in metadata_paths if path.is_file()),
        default=0,
    )
    if cache_path.is_file():
        try:
            with np.load(cache_path, allow_pickle=False) as cached:
                valid = (
                    int(cached["version"]) == WBC_CACHE_VERSION
                    and int(cached["tick_size"]) == tick_stat.st_size
                    and int(cached["tick_mtime_ns"]) == tick_stat.st_mtime_ns
                    and int(cached["metadata_mtime_ns"]) == metadata_mtime_ns
                )
                if valid:
                    return {
                        "time": cached["time"].copy(),
                        "start": float(cached["start"]),
                        "end": float(cached["end"]),
                        "errors": cached["errors"].copy(),
                        "position": cached["position"].copy(),
                        "orientation": cached["orientation"].copy(),
                    }
        except (OSError, KeyError, ValueError):
            pass

    loaded = load_tick(tick_path)
    if loaded is None:
        return None
    t, opt_x, meas_rbd, _planned_mode = loaded
    if t.size < 5:
        return None
    time = t - t[0]
    start, end = command_window_in_tick_time(trial, float(time[-1]))
    if end <= start:
        return None
    errors = np.asarray(compute_errors(opt_x, meas_rbd))
    position = meas_rbd[:, 3:6] - opt_x[:, 6:9]
    orientation_raw = meas_rbd[:, 0:3] - opt_x[:, 9:12]
    orientation = np.arctan2(np.sin(orientation_raw), np.cos(orientation_raw))
    analysis = {
        "time": time,
        "start": start,
        "end": end,
        "errors": errors,
        "position": position,
        "orientation": orientation,
    }
    temporary_path = cache_path.with_suffix(".npz.tmp")
    try:
        with temporary_path.open("wb") as stream:
            np.savez_compressed(
                stream,
                version=np.asarray(WBC_CACHE_VERSION, dtype=np.int64),
                tick_size=np.asarray(tick_stat.st_size, dtype=np.int64),
                tick_mtime_ns=np.asarray(tick_stat.st_mtime_ns, dtype=np.int64),
                metadata_mtime_ns=np.asarray(metadata_mtime_ns, dtype=np.int64),
                time=time,
                start=np.asarray(start),
                end=np.asarray(end),
                errors=errors,
                position=position,
                orientation=orientation,
            )
        temporary_path.replace(cache_path)
    except OSError:
        if temporary_path.is_file():
            temporary_path.unlink()
    return analysis


def load_trial_errors(trial: dict):
    analysis = load_trial_wbc_analysis(trial)
    if analysis is None:
        return None
    return (
        analysis["time"],
        analysis["start"],
        analysis["end"],
        tuple(analysis["errors"]),
    )


def aggregate_condition(trials: list[dict]):
    """Interpolate a condition's trials and return mean/std time series."""
    loaded = []
    for trial in trials:
        record = load_trial_errors(trial)
        if record is None:
            print(f"[warn] unusable tick.csv: {trial['trial_dir'].name}", file=sys.stderr)
            continue
        loaded.append((trial, record))
    if not loaded:
        return None

    # Keep the full command window even when one failed trial ends early. Missing
    # tails remain NaN and do not flatten the aggregate through interpolation.
    duration = max(end - start for _trial, (_time, start, end, _errors) in loaded)
    if duration <= 0.0:
        return None
    display_grid = np.arange(0.0, duration + 1e-9, 0.005)
    output = {"t_grid": display_grid, "duration": duration, "n": len(loaded)}
    metric_names = ("base_pos", "base_orientation", "joint_position")
    for metric_index, metric_name in enumerate(metric_names):
        stack = []
        for _trial, (time, start, _end, errors) in loaded:
            query = display_grid + start
            values = np.full(display_grid.shape, np.nan)
            valid = (query >= time[0]) & (query <= min(time[-1], _end))
            values[valid] = np.interp(query[valid], time, errors[metric_index])
            stack.append(values)
        values = np.asarray(stack)
        output[f"{metric_name}_mean"] = np.nanmean(values, axis=0)
        output[f"{metric_name}_std"] = np.nanstd(values, axis=0)
        if metric_index == 0:
            output["sample_count"] = np.sum(np.isfinite(values), axis=0)
    descent_starts, descent_completes = [], []
    for trial, record in loaded:
        _time, command_start, _command_end, _errors = record
        event_start, event_complete = descent_times(load_or_detect_events(trial["trial_dir"]))
        if event_start is not None:
            display_time = event_start - command_start
            if 0.0 <= display_time <= duration:
                descent_starts.append(display_time)
        if event_complete is not None:
            display_time = event_complete - command_start
            if 0.0 <= display_time <= duration:
                descent_completes.append(display_time)
    for name, values in (
        ("descent_start", descent_starts),
        ("descent_complete", descent_completes),
    ):
        output[f"{name}_mean"] = float(np.mean(values)) if values else None
        output[f"{name}_std"] = float(np.std(values)) if values else None
        output[f"{name}_n"] = len(values)
    return output


def select_cohort(trials: list[dict], cohort: str) -> list[dict]:
    if cohort == "successful_only":
        return [trial for trial in trials if bool(trial.get("result", {}).get("success"))]
    if cohort == "max_contrast_successful_pair":
        return select_max_contrast_successful_pair(trials)[0]
    return trials


def trial_tracking_metrics(trial: dict):
    record = load_trial_errors(trial)
    if record is None:
        return None
    time, start, end, errors = record
    mask = (time >= start) & (time <= end)
    if np.count_nonzero(mask) < 2:
        return None
    return np.asarray([float(np.mean(metric[mask])) for metric in errors])


def select_max_contrast_successful_pair(trials: list[dict]):
    """Select best successful ON and worst successful OFF at each offset.

    Each trial gets a dimensionless composite score: its three time-mean
    squared tracking errors divided by the pooled successful-trial median for
    the corresponding metric, then averaged. This keeps unlike units from
    directly dominating the selection.
    """
    successful = [trial for trial in trials if bool(trial.get("result", {}).get("success"))]
    selected, report_conditions = [], []
    for offset in sorted({trial["offset_m"] for trial in successful}):
        candidates = []
        for trial in successful:
            if trial["offset_m"] != offset:
                continue
            metrics = trial_tracking_metrics(trial)
            if metrics is not None and np.all(np.isfinite(metrics)):
                candidates.append((trial, metrics))
        if not candidates:
            continue
        normalizer = np.median(np.asarray([metrics for _trial, metrics in candidates]), axis=0)
        normalizer = np.maximum(normalizer, 1e-12)
        scored = [
            (trial, metrics, float(np.mean(metrics / normalizer)))
            for trial, metrics in candidates
        ]
        on_candidates = [item for item in scored if item[0]["robust"] == "ON"]
        off_candidates = [item for item in scored if item[0]["robust"] == "OFF"]
        if not on_candidates or not off_candidates:
            continue
        on_selected = min(on_candidates, key=lambda item: (item[2], item[0]["trial_dir"].name))
        off_selected = max(off_candidates, key=lambda item: (item[2], item[0]["trial_dir"].name))
        selected.extend((on_selected[0], off_selected[0]))
        report_conditions.append(
            {
                "offset_m": offset,
                "normalizer": {
                    "base_position_squared_norm_m2": float(normalizer[0]),
                    "base_orientation_squared_norm_deg2": float(normalizer[1]),
                    "joint_position_squared_norm_rad2": float(normalizer[2]),
                },
                "selected": {
                    "ON_best": {
                        "trial": on_selected[0]["trial_dir"].name,
                        "score": on_selected[2],
                        "metrics": on_selected[1].tolist(),
                    },
                    "OFF_worst": {
                        "trial": off_selected[0]["trial_dir"].name,
                        "score": off_selected[2],
                        "metrics": off_selected[1].tolist(),
                    },
                },
            }
        )
    return selected, {
        "selection": "successful Robust ON minimum score vs successful Robust OFF maximum score",
        "score": "mean(metric / pooled-successful median), using three time-mean squared WBC tracking errors",
        "warning": "Deliberately maximized contrast; do not interpret as an unbiased aggregate result.",
        "conditions": report_conditions,
    }


def plot_cohort(trials: list[dict], cohort: str, output_path: Path):
    groups = group_trials(trials)
    if not groups:
        raise RuntimeError(f"No WBC-compatible trials found for cohort={cohort}")

    aggregated = {}
    for key, grouped in groups.items():
        aggregate = aggregate_condition(grouped)
        if aggregate is not None:
            aggregated[key] = aggregate
    if not aggregated:
        raise RuntimeError(f"No usable WBC tracking data found for cohort={cohort}")

    offsets = sorted({key[0] for key in aggregated})
    fig, axes = plt.subplots(
        3, len(offsets), figsize=paper_figsize(len(offsets), 3), squeeze=False
    )
    metric_meta = (
        ("base_pos", r"$\Vert e_p\Vert_2^2$ [m$^2$]"),
        ("base_orientation", r"$\Vert e_R\Vert_2^2$ [deg$^2$]"),
        ("joint_position", r"$\Vert e_q\Vert_2^2$ [rad$^2$]"),
    )
    condition_styles = {
        "ON": {"color": PROPOSED_COLOR, "label": "Proposed"},
        "OFF": {"color": NOMINAL_COLOR, "label": "Baseline"},
    }
    event_summary_label = "selected" if cohort == "max_contrast_successful_pair" else "mean"
    for column, offset in enumerate(offsets):
        offset_data = [
            aggregated[(offset, robust)]
            for robust in condition_styles
            if (offset, robust) in aggregated
        ]

        def pooled_event_mean(event_name: str):
            weighted_sum = 0.0
            sample_count = 0
            for data in offset_data:
                mean = data.get(f"{event_name}_mean")
                count = int(data.get(f"{event_name}_n", 0))
                if mean is not None and count > 0:
                    weighted_sum += float(mean) * count
                    sample_count += count
            return weighted_sum / sample_count if sample_count else None

        descent_start = pooled_event_mean("descent_start")
        descent_complete = pooled_event_mean("descent_complete")
        durations = [
            aggregated[(offset, robust)]["duration"]
            for robust in condition_styles
            if (offset, robust) in aggregated
        ]
        max_duration = max(durations)
        for row, (metric, ylabel) in enumerate(metric_meta):
            axis = axes[row, column]
            for robust, style in condition_styles.items():
                data = aggregated.get((offset, robust))
                if data is None:
                    continue
                time = data["t_grid"]
                mean = data[f"{metric}_mean"]
                std = data[f"{metric}_std"]
                axis.plot(
                    time,
                    mean,
                    color=style["color"],
                    label=style["label"],
                    linewidth=2.25,
                )
                axis.fill_between(
                    time, mean - std, mean + std, color=style["color"], alpha=0.18, linewidth=0
                )
            if descent_start is not None:
                axis.axvline(
                    descent_start,
                    color=DESCENT_START_COLOR,
                    linestyle="--",
                    linewidth=1.1,
                    label=None,
                )
            if descent_complete is not None:
                axis.axvline(
                    descent_complete,
                    color=DESCENT_COMPLETE_COLOR,
                    linestyle=":",
                    linewidth=1.0,
                    label=None,
                )
            if row == 0:
                axis.set_title(offset_label(offset))
            if column == 0:
                axis.set_ylabel(ylabel)
            if row == 2:
                axis.set_xlabel("Command-active time [s]")
            axis.set_xlim(0, max_duration)
            style_paper_axis(axis)

    cohort_title = {
        "all_trials": "all trials",
        "successful_only": "successful trials only",
        "max_contrast_successful_pair": "max-contrast successful pair (selected)",
    }[cohort]
    fig.suptitle(
        f"WBC Tracking Error - {cohort_title.title()}",
        y=0.995,
    )
    legend_handles = [
        Line2D([], [], color=PROPOSED_COLOR, linewidth=2.25, label="Proposed mean"),
        Line2D([], [], color=NOMINAL_COLOR, linewidth=2.25, label="Baseline mean"),
        Line2D([], [], color=DESCENT_START_COLOR, linestyle="--", label="First lower touchdown"),
        Line2D([], [], color=DESCENT_COMPLETE_COLOR, linestyle=":", label="Descent complete"),
    ]
    paper_legend(
        fig, handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 0.965),
        ncol=2,
    )
    apply_figure_font_sizes(fig, output_path)
    fig.tight_layout(rect=(0, 0, 1, 0.89))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=SAVE_DPI)
    plt.close(fig)
    print(f"saved {output_path}")

    print(f"\n=== {cohort_title}: time-mean squared WBC tracking error ===")
    for (offset, robust), data in sorted(aggregated.items()):
        summaries = [
            float(np.nanmean(data[f"{metric}_mean"]))
            for metric in ("base_pos", "base_orientation", "joint_position")
        ]
        print(
            f"{offset_label(offset):10s} {robust:3s} n={data['n']}: "
            f"base={summaries[0]:.6g} m², orientation={summaries[1]:.6g} deg², "
            f"joint={summaries[2]:.6g} rad²"
        )
        if data.get("descent_start_mean") is not None:
            print(
                f"  {event_summary_label} lower touchdown={data['descent_start_mean']:.3f} "
                f"± {data['descent_start_std']:.3f} s; "
                f"{event_summary_label} complete={data.get('descent_complete_mean')} s"
            )
    return output_path


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
    trials = discover_trials(results_dir)
    if not trials:
        raise RuntimeError(f"No WBC-compatible trials found in {results_dir}")
    cohorts = (
        ("all_trials", "successful_only", "max_contrast_successful_pair")
        if args.cohort == "all"
        else (args.cohort,)
    )
    for cohort in cohorts:
        cohort_trials = select_cohort(trials, cohort)
        plot_cohort(cohort_trials, cohort, out_dir / f"wbc_tracking_error_{cohort}.png")
        if cohort == "max_contrast_successful_pair":
            _selected, report = select_max_contrast_successful_pair(trials)
            (out_dir / "wbc_max_contrast_selection.json").write_text(
                json.dumps(report, indent=2), encoding="utf-8"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
