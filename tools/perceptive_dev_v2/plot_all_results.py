#!/usr/bin/env python3
"""Generate a complete gallery and aggregate dashboard for every saved trial."""

import argparse
import csv
import html
import json
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from monte_carlo_dashboard import MC_TAG_RE, generate_monte_carlo_outputs
from paper_plot_style import (
    FAILURE_COLOR,
    NOMINAL_COLOR,
    PROPOSED_COLOR,
    SAVE_DPI,
    apply_paper_style,
    paper_figsize,
    paper_legend,
    style_paper_axis,
)
from plot_mpc_timing import parse_mpc_timing
from trial_metrics import analyze_trial


apply_paper_style()


ROOT = Path(__file__).resolve().parent
DEFAULT_RESULTS = ROOT / "results"
DASHBOARD_METRICS = (
    ("success", "Success [%]", True),
    ("body_frame_forward_progress", "Progress [m]", True),
    ("body_frame_lateral_progress", "Lateral drift [m]", False),
    ("roll_rms_deg", "Roll RMS [deg]", False),
    ("pitch_rms_deg", "Pitch RMS [deg]", False),
    ("yaw_rms_deg", "Yaw RMS [deg]", False),
    ("base_position_rmse_m", "Base pos. RMSE [m]", False),
    ("base_orientation_rmse_deg", "Base ori. RMSE [deg]", False),
    ("joint_position_rmse_rad", "Joint RMSE [rad]", False),
    ("touchdown_normal_speed_mean_mps", "Touchdown normal speed [m/s]", False),
)
MPC_DASHBOARD_METRICS = (
    ("mpc_solve_ms_avg", "Mean solve time [ms]"),
    ("mpc_solve_ms_p95", "P95 solve time [ms]"),
    ("mpc_solve_hz_avg", "Achieved solve rate [Hz]"),
    ("mpc_capacity_hz_avg", "Solver capacity [Hz]"),
    ("mpc_deadline_rate_pct", "MPC deadline satisfaction [%]"),
    ("sqp_iterations_avg", "Mean SQP iterations [count]"),
    ("wbc_solve_ms_mean", "Mean WBC compute time [ms]"),
    ("wbc_achieved_hz", "Achieved WBC rate [Hz]"),
    ("wbc_deadline_rate_pct", "WBC deadline satisfaction [%]"),
)
ROBUST_DASHBOARD_METRICS = (
    ("robust_band_traversal_rate_pct", "Robust-band traversal [%]"),
    ("uncertainty_coverage_ratio", "Uncertainty coverage ratio"),
    ("normalized_boundary_violation", "Normalized boundary violation"),
    ("contact_in_band_rate_pct", "Contact-in-band rate [%]"),
)
INDIVIDUAL_PLOT_PRODUCTS = {
    "plot_trial_rmse.py": {
        "inputs": ("tick.csv",),
        "outputs": ("tracking_rmse.png", "tracking_rmse.json"),
    },
    "plot_robust_phase.py": {
        "inputs": ("tick.csv", "controller.log"),
        "outputs": ("robust_phase_foot_z.png",),
    },
    "plot_mpc_timing.py": {
        "inputs": ("controller.log",),
        "outputs": ("mpc_timing.png", "mpc_timing.json"),
    },
}


def infer_metadata(trial_dir: Path, result: dict) -> dict:
    name = trial_dir.name
    config = {}
    config_path = trial_dir / "run_config.json"
    if config_path.is_file():
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass

    effective = config.get("effective_task_parameters", {}).get("robustPhase", {})
    enabled = effective.get("enabled")
    if enabled is None:
        robust = "ON" if re.search(r"(?:^|_)ON(?:_|$)|robON", name) else "OFF"
    else:
        robust = "ON" if str(enabled).lower() == "true" else "OFF"

    offset = config.get("terrain_z_offset")
    if offset is None:
        match = re.search(r"off([MP])(\d+)", name)
        if match:
            magnitude = int(match.group(2)) / (10 ** len(match.group(2)))
            offset = magnitude if match.group(1) == "P" else -magnitude
        elif re.search(r"(?:^|_)off0(?:_|$)", name):
            offset = 0.0

    run_match = re.search(r"_run(\d+)(?:_|$)", name)
    run = int(run_match.group(1)) if run_match else None
    splice = effective.get("enable_splice")
    if splice is None:
        splice = "OFF" if "nosplice" in name.lower() else "unknown"
    else:
        splice = "ON" if str(splice).lower() == "true" else "OFF"

    return {
        "trial": name,
        "offset_m": float(offset) if offset is not None else None,
        "robust": robust,
        "splice": splice,
        "run": run,
        "scenario": result.get("scenario", config.get("scenario", "unknown")),
    }


def run_plot(script: Path, trial_dir: Path) -> tuple[bool, str]:
    proc = subprocess.run(
        [sys.executable, str(script), str(trial_dir)],
        capture_output=True,
        text=True,
        timeout=240.0,
    )
    detail = (proc.stdout if proc.returncode == 0 else proc.stderr or proc.stdout).strip()
    return proc.returncode == 0, detail


def individual_plot_is_current(script_name: str, trial_dir: Path) -> bool:
    """Return true when immutable trial inputs already have complete outputs."""
    product = INDIVIDUAL_PLOT_PRODUCTS[script_name]
    inputs = [trial_dir / name for name in product["inputs"]]
    outputs = [trial_dir / name for name in product["outputs"]]
    if not all(path.is_file() and path.stat().st_size > 0 for path in inputs + outputs):
        return False
    newest_input = max(path.stat().st_mtime_ns for path in inputs)
    oldest_output = min(path.stat().st_mtime_ns for path in outputs)
    return oldest_output >= newest_input


def load_trials(results_dir: Path) -> tuple[list[dict], list[dict]]:
    valid = []
    skipped = []
    for trial_dir in sorted(path for path in results_dir.iterdir() if path.is_dir()):
        if trial_dir.name == "all_visualizations":
            continue
        result_path = trial_dir / "result.json"
        tick_path = trial_dir / "tick.csv"
        if not result_path.is_file() or not tick_path.is_file() or tick_path.stat().st_size == 0:
            skipped.append({"trial": trial_dir.name, "reason": "missing result.json or tick.csv"})
            continue
        try:
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            skipped.append({"trial": trial_dir.name, "reason": f"invalid result.json: {exc}"})
            continue
        record = infer_metadata(trial_dir, result)
        record.update(result)
        record["trial_dir"] = trial_dir
        valid.append(record)
    return valid, skipped


def attach_rmse(records: list[dict]):
    for record in records:
        path = record["trial_dir"] / "tracking_rmse.json"
        if not path.is_file():
            continue
        try:
            rmse = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        record["base_position_rmse_m"] = rmse.get("base_position_rmse_m")
        record["base_orientation_rmse_deg"] = rmse.get("base_orientation_rmse_deg")
        record["joint_position_rmse_rad"] = rmse.get("joint_position_rmse_rad")


def attach_terrain_descent(records: list[dict]):
    for record in records:
        path = record["trial_dir"] / "terrain_descent_events.json"
        if not path.is_file():
            continue
        try:
            descent = json.loads(path.read_text(encoding="utf-8")).get("descent", {})
        except (OSError, json.JSONDecodeError):
            continue
        start = descent.get("first_lower_touchdown")
        complete = descent.get("last_hind_lower_touchdown")
        record["terrain_descent_start_tick_sec"] = (
            start.get("tick_relative_sec") if start else None
        )
        record["terrain_descent_complete_tick_sec"] = (
            complete.get("tick_relative_sec") if complete else None
        )
        record["terrain_descent_duration_sec"] = descent.get("duration_sec")


def attach_mpc_timing(records: list[dict]):
    for record in records:
        path = record["trial_dir"] / "controller.log"
        if not path.is_file():
            continue
        try:
            timing = parse_mpc_timing(path)
        except OSError:
            continue
        if timing["timestamp"].size == 0:
            continue
        record["mpc_solve_ms_avg"] = float(np.mean(timing["solve_ms_avg"]))
        record["mpc_solve_ms_p95"] = float(np.percentile(timing["solve_ms_last"], 95))
        record["mpc_solve_hz_avg"] = float(np.mean(timing["solve_hz"]))
        record["mpc_capacity_hz_avg"] = float(np.mean(timing["capacity_hz"]))
        record["mpc_load_pct_avg"] = float(np.mean(timing["load_pct"]))
        valid_iterations = np.isfinite(timing["sqp_iter_avg"])
        if np.any(valid_iterations):
            record["sqp_iterations_avg"] = float(
                np.mean(timing["sqp_iter_avg"][valid_iterations])
            )
        exact = np.isfinite(timing["deadline_hits"]) & np.isfinite(
            timing["deadline_solves"]
        )
        if np.any(exact) and np.sum(timing["deadline_solves"][exact]) > 0:
            record["mpc_deadline_rate_pct"] = float(
                100.0 * np.sum(timing["deadline_hits"][exact])
                / np.sum(timing["deadline_solves"][exact])
            )
            record["mpc_deadline_method"] = "exact_per_solve"
        else:
            deadline_ms = np.divide(
                1000.0,
                timing["target_hz"],
                out=np.full_like(timing["target_hz"], np.nan),
                where=timing["target_hz"] > 0.0,
            )
            valid = np.isfinite(deadline_ms) & np.isfinite(timing["solve_ms_last"])
            if np.any(valid):
                record["mpc_deadline_rate_pct"] = float(
                    100.0 * np.mean(timing["solve_ms_last"][valid] <= deadline_ms[valid])
                )
                record["mpc_deadline_method"] = "legacy_last_sample_estimate"


def attach_trial_metrics(records: list[dict]) -> list[dict]:
    failures = []
    scalar_runtime_fields = (
        "contact_source",
        "touchdown_event_count",
        "touchdown_normal_speed_mean_mps",
        "touchdown_normal_speed_p95_mps",
        "wbc_target_hz",
        "wbc_achieved_hz",
        "wbc_tick_period_mean_ms",
        "wbc_tick_period_p95_ms",
        "wbc_solve_ms_mean",
        "wbc_solve_ms_p95",
        "wbc_deadline_rate_pct",
        "wbc_deadline_method",
    )
    scalar_robust_fields = tuple(metric for metric, _label in ROBUST_DASHBOARD_METRICS)
    for record in records:
        try:
            runtime, robust = analyze_trial(record["trial_dir"])
        except (OSError, RuntimeError, ValueError) as exc:
            failures.append(
                {"trial": record["trial"], "plot": "trial_metrics", "error": str(exc)}
            )
            continue
        for field in scalar_runtime_fields:
            record[field] = runtime.get(field)
        record["robust_metrics_available"] = robust.get("available", False)
        record["robust_metric_contact_sources"] = ",".join(
            robust.get("contact_sources", [])
        )
        record["robust_matched_touchdown_count"] = robust.get("matched_touchdown_count")
        for field in scalar_robust_fields:
            record[field] = robust.get(field)
    return failures


def group_label(record: dict) -> str:
    offset = record.get("offset_m")
    offset_label = "unknown" if offset is None else rf"$\Delta z={offset:+.2f}$"
    method = "Proposed" if record["robust"] == "ON" else "Baseline"
    return f"{offset_label}\n{method}"


def group_key(record: dict):
    offset = record.get("offset_m")
    return (float("inf") if offset is None else offset, 0 if record["robust"] == "ON" else 1)


def select_exp_foothold_pair(
    records: list[dict], target_offset_m: float = 0.03, target_run: int = 1
) -> dict[str, dict]:
    """Select the latest adjacent successful ON/OFF pair at one exact error."""
    candidates = [
        record for record in records
        if record.get("success")
        and record.get("run") == target_run
        and record.get("offset_m") is not None
        and abs(float(record["offset_m"]) - target_offset_m) <= 1e-9
        and record.get("robust") in {"ON", "OFF"}
        and (record["trial_dir"] / "controller.log").is_file()
    ]
    by_mode = {
        mode: [record for record in candidates if record.get("robust") == mode]
        for mode in ("ON", "OFF")
    }
    if not by_mode["ON"] or not by_mode["OFF"]:
        return {}

    def timestamp(record):
        try:
            return datetime.strptime(record["trial"][:15], "%Y%m%d_%H%M%S")
        except ValueError:
            return datetime.min

    possible_pairs = [
        (on_record, off_record)
        for on_record in by_mode["ON"]
        for off_record in by_mode["OFF"]
        if abs((timestamp(on_record) - timestamp(off_record)).total_seconds()) <= 300.0
    ]
    if not possible_pairs:
        return {}
    on_record, off_record = max(
        possible_pairs,
        key=lambda pair: (
            min(timestamp(pair[0]), timestamp(pair[1])),
            -abs((timestamp(pair[0]) - timestamp(pair[1])).total_seconds()),
        ),
    )
    return {"ON": on_record, "OFF": off_record}


def metric_value(record: dict, key: str):
    value = record.get(key)
    if value is None:
        return None
    value = float(value)
    return abs(value) if key == "body_frame_lateral_progress" else value


def plot_dashboard(records: list[dict], output: Path):
    groups = defaultdict(list)
    for record in records:
        groups[group_key(record)].append(record)
    keys = sorted(groups)
    labels = [group_label(groups[key][0]) for key in keys]
    colors = [PROPOSED_COLOR if groups[key][0]["robust"] == "ON" else NOMINAL_COLOR for key in keys]

    columns = 3
    rows = int(np.ceil(len(DASHBOARD_METRICS) / columns))
    fig, axes = plt.subplots(
        rows, columns, figsize=paper_figsize(columns, rows), squeeze=False
    )
    rng = np.random.default_rng(7)
    for axis, (metric, column_label, _higher_is_better) in zip(axes.flat, DASHBOARD_METRICS):
        title, unit = column_label.rsplit(" [", 1)
        unit = unit.rstrip("]")
        means, stds = [], []
        values_by_group = []
        for key in keys:
            if metric == "success":
                values = [100.0 if bool(r.get("success")) else 0.0 for r in groups[key]]
            else:
                values = [metric_value(r, metric) for r in groups[key]]
                values = [value for value in values if value is not None and np.isfinite(value)]
            values_by_group.append(values)
            means.append(float(np.mean(values)) if values else np.nan)
            stds.append(float(np.std(values)) if len(values) > 1 else 0.0)
        if not any(values_by_group):
            title, unit = column_label.rsplit(" [", 1)
            axis.set_title(title)
            axis.set_ylabel(unit.rstrip("]"))
            axis.text(
                0.5, 0.5, "Unavailable in legacy logs\n(recorded by new trials)",
                ha="center", va="center", transform=axis.transAxes, color="dimgray"
            )
            axis.set_xticks([])
            axis.grid(False)
            continue
        x = np.arange(len(keys))
        axis.bar(
            x, means, yerr=stds, color=colors, alpha=0.86, capsize=4,
            edgecolor="0.25", linewidth=0.8,
        )
        for index, (key, values) in enumerate(zip(keys, values_by_group)):
            for record, value in zip(groups[key], values):
                jitter = float(rng.uniform(-0.09, 0.09))
                failed = not bool(record.get("success"))
                axis.scatter(
                    index + jitter,
                    value,
                    color=FAILURE_COLOR if failed else "black",
                    marker="x" if failed else "o",
                    s=38,
                    zorder=4,
                )
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.set_xticks(x, labels)
        style_paper_axis(axis, grid_axis="y")
        if metric == "success":
            axis.set_ylim(0, 110)
    for axis in axes.flat[len(DASHBOARD_METRICS):]:
        axis.set_visible(False)
    legend_handles = [
        Patch(facecolor=PROPOSED_COLOR, edgecolor="0.25", label="Proposed"),
        Patch(facecolor=NOMINAL_COLOR, edgecolor="0.25", label="Baseline"),
        Line2D([], [], color="black", marker="o", linestyle="None", label="Successful trial"),
        Line2D([], [], color=FAILURE_COLOR, marker="x", linestyle="None", label="Failed trial"),
    ]
    fig.suptitle("Perceptive Robust-Phase Performance", y=0.995)
    paper_legend(
        fig, handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 0.965),
        ncol=4,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(output, dpi=SAVE_DPI)
    plt.close(fig)


def plot_mpc_dashboard(records: list[dict], output: Path):
    groups = defaultdict(list)
    for record in records:
        groups[group_key(record)].append(record)
    keys = sorted(groups)
    labels = [group_label(groups[key][0]) for key in keys]
    colors = [PROPOSED_COLOR if groups[key][0]["robust"] == "ON" else NOMINAL_COLOR for key in keys]

    columns = 3
    rows = int(np.ceil(len(MPC_DASHBOARD_METRICS) / columns))
    fig, axes = plt.subplots(
        rows, columns, figsize=paper_figsize(columns, rows), squeeze=False
    )
    rng = np.random.default_rng(11)
    for axis, (metric, label) in zip(axes.flat, MPC_DASHBOARD_METRICS):
        values_by_group = []
        means, stds = [], []
        for key in keys:
            values = [record.get(metric) for record in groups[key]]
            values = [float(value) for value in values if value is not None and np.isfinite(value)]
            values_by_group.append(values)
            means.append(float(np.mean(values)) if values else np.nan)
            stds.append(float(np.std(values)) if len(values) > 1 else 0.0)
        if not any(values_by_group):
            title, unit = label.rsplit(" [", 1)
            axis.set_title(title)
            axis.set_ylabel(unit.rstrip("]"))
            axis.text(
                0.5, 0.5, "Unavailable in legacy logs\n(recorded by new trials)",
                ha="center", va="center", transform=axis.transAxes, color="dimgray"
            )
            axis.set_xticks([])
            axis.grid(False)
            continue
        x = np.arange(len(keys))
        axis.bar(
            x, means, yerr=stds, color=colors, alpha=0.86, capsize=4,
            edgecolor="0.25", linewidth=0.8,
        )
        for index, (key, values) in enumerate(zip(keys, values_by_group)):
            records_with_value = [
                record for record in groups[key]
                if record.get(metric) is not None and np.isfinite(float(record[metric]))
            ]
            for record, value in zip(records_with_value, values):
                failed = not bool(record.get("success"))
                axis.scatter(
                    index + float(rng.uniform(-0.09, 0.09)),
                    value,
                    color=FAILURE_COLOR if failed else "black",
                    marker="x" if failed else "o",
                    s=38,
                    zorder=4,
                )
        title, unit = label.rsplit(" [", 1)
        axis.set_title(title)
        axis.set_ylabel(unit.rstrip("]"))
        axis.set_xticks(x, labels)
        style_paper_axis(axis, grid_axis="y")
        if metric.endswith("deadline_rate_pct"):
            axis.set_ylim(0, 105)
    for axis in axes.flat[len(MPC_DASHBOARD_METRICS):]:
        axis.set_visible(False)
    legend_handles = [
        Patch(facecolor=PROPOSED_COLOR, edgecolor="0.25", label="Proposed"),
        Patch(facecolor=NOMINAL_COLOR, edgecolor="0.25", label="Baseline"),
        Line2D([], [], color="black", marker="o", linestyle="None", label="Successful trial"),
        Line2D([], [], color=FAILURE_COLOR, marker="x", linestyle="None", label="Failed trial"),
    ]
    fig.suptitle("MPC/WBC Timing Performance", y=0.995)
    paper_legend(
        fig, handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
        ncol=4,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(output, dpi=SAVE_DPI)
    plt.close(fig)


def plot_robust_dashboard(records: list[dict], output: Path):
    groups = defaultdict(list)
    for record in records:
        if record.get("robust_metrics_available"):
            groups[group_key(record)].append(record)
    if not groups:
        return False
    keys = sorted(groups)
    labels = [group_label(groups[key][0]) for key in keys]
    fig, axes = plt.subplots(2, 2, figsize=paper_figsize(2, 2))
    rng = np.random.default_rng(19)
    for axis, (metric, label) in zip(axes.flat, ROBUST_DASHBOARD_METRICS):
        values_by_group = []
        means, stds = [], []
        for key in keys:
            values = [record.get(metric) for record in groups[key]]
            values = [float(value) for value in values if value is not None and np.isfinite(value)]
            values_by_group.append(values)
            means.append(float(np.mean(values)) if values else np.nan)
            stds.append(float(np.std(values)) if len(values) > 1 else 0.0)
        x = np.arange(len(keys))
        colors = [PROPOSED_COLOR if groups[key][0]["robust"] == "ON" else NOMINAL_COLOR for key in keys]
        axis.bar(
            x, means, yerr=stds, color=colors, alpha=0.86, capsize=4,
            edgecolor="0.25", linewidth=0.8,
        )
        for index, values in enumerate(values_by_group):
            for value in values:
                axis.scatter(index + float(rng.uniform(-0.09, 0.09)), value,
                             color="black", s=38, zorder=4)
        title, unit = label.rsplit(" [", 1) if " [" in label else (label, "ratio")
        axis.set_title(title)
        axis.set_ylabel(unit.rstrip("]"))
        axis.set_xticks(x, labels)
        style_paper_axis(axis, grid_axis="y")
        if metric.endswith("_pct"):
            axis.set_ylim(0, 105)
    present_methods = {groups[key][0]["robust"] for key in keys}
    legend_handles = []
    if "ON" in present_methods:
        legend_handles.append(
            Patch(facecolor=PROPOSED_COLOR, edgecolor="0.25", label="Proposed")
        )
    if "OFF" in present_methods:
        legend_handles.append(
            Patch(facecolor=NOMINAL_COLOR, edgecolor="0.25", label="Baseline")
        )
    legend_handles.append(
        Line2D([], [], color="black", marker="o", linestyle="None", label="Individual trial")
    )
    fig.suptitle("Robust-Phase Uncertainty Metrics", y=0.995)
    paper_legend(
        fig, handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 0.945),
        ncol=len(legend_handles),
    )
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(output, dpi=SAVE_DPI)
    plt.close(fig)
    return True


def dashboard_metric_mean(records: list[dict], metric: str):
    if metric == "success":
        values = [100.0 if bool(record.get("success")) else 0.0 for record in records]
    else:
        values = [metric_value(record, metric) for record in records]
        values = [value for value in values if value is not None and np.isfinite(value)]
    return float(np.mean(values)) if values else None


def terrain_error_label(offset_m: float) -> str:
    centimeters = offset_m * 100.0
    if abs(centimeters) < 1e-9:
        return "0 cm"
    if abs(centimeters - round(centimeters)) < 1e-9:
        return f"{round(centimeters):+d} cm"
    return f"{centimeters:+g} cm"


def dashboard_method_row(
    terrain_error: str, method: str, records: list[dict], robust: str
) -> dict:
    method_records = [record for record in records if record.get("robust") == robust]
    return {
        "Terrain error": terrain_error,
        "Method": method,
        **{
            label: dashboard_metric_mean(method_records, metric)
            for metric, label, _higher_is_better in DASHBOARD_METRICS
        },
    }


def write_dashboard_comparison_table(records: list[dict], output: Path):
    """Write terrain-separated and overall Baseline/Proposed dashboard means."""
    rows = []
    offsets = sorted(
        {float(record["offset_m"]) for record in records if record.get("offset_m") is not None}
    )
    for offset in offsets:
        offset_records = [
            record
            for record in records
            if record.get("offset_m") is not None
            and abs(float(record["offset_m"]) - offset) < 1e-9
        ]
        label = terrain_error_label(offset)
        rows.append(dashboard_method_row(label, "Baseline", offset_records, "OFF"))
        rows.append(dashboard_method_row(label, "Proposed", offset_records, "ON"))

    baseline = dashboard_method_row("Overall", "Baseline", records, "OFF")
    proposed = dashboard_method_row("Overall", "Proposed", records, "ON")
    rows.extend((baseline, proposed))
    improvement = {"Terrain error": "Overall", "Method": "Improvement (%)"}
    for _metric, label, higher_is_better in DASHBOARD_METRICS:
        baseline_value = baseline[label]
        proposed_value = proposed[label]
        if baseline_value is None or proposed_value is None or abs(baseline_value) < 1e-12:
            improvement[label] = None
            continue
        difference = (
            proposed_value - baseline_value
            if higher_is_better
            else baseline_value - proposed_value
        )
        improvement[label] = difference / abs(baseline_value) * 100.0
    rows.append(improvement)

    fields = [
        "Terrain error",
        "Method",
        *(label for _metric, label, _higher in DASHBOARD_METRICS),
    ]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    field: (
                        row.get(field)
                        if field in {"Terrain error", "Method"} or row.get(field) is None
                        else f"{row[field]:.6f}"
                    )
                    for field in fields
                }
            )


def write_summary(records: list[dict], skipped: list[dict], output_dir: Path):
    fields = [
        "trial", "offset_m", "robust", "splice", "run", "scenario", "success",
        "fall_reason", "time_to_failure", "duration_executed",
        "body_frame_forward_progress", "body_frame_lateral_progress",
        "roll_rms_deg", "pitch_rms_deg", "yaw_rms_deg", "min_base_z", "base_z_std",
        "base_position_rmse_m", "base_orientation_rmse_deg", "joint_position_rmse_rad",
        "terrain_descent_start_tick_sec", "terrain_descent_complete_tick_sec",
        "terrain_descent_duration_sec",
        "mpc_solve_ms_avg", "mpc_solve_ms_p95", "mpc_solve_hz_avg",
        "mpc_capacity_hz_avg", "mpc_load_pct_avg",
        "mpc_deadline_rate_pct", "mpc_deadline_method", "sqp_iterations_avg",
        "contact_source", "touchdown_event_count", "touchdown_normal_speed_mean_mps",
        "touchdown_normal_speed_p95_mps", "wbc_target_hz", "wbc_achieved_hz",
        "wbc_tick_period_mean_ms", "wbc_tick_period_p95_ms", "wbc_solve_ms_mean",
        "wbc_solve_ms_p95", "wbc_deadline_rate_pct", "wbc_deadline_method",
        "robust_metrics_available", "robust_metric_contact_sources",
        "robust_matched_touchdown_count", "robust_band_traversal_rate_pct",
        "uncertainty_coverage_ratio", "normalized_boundary_violation",
        "contact_in_band_rate_pct",
    ]
    with (output_dir / "all_trials_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({field: record.get(field) for field in fields})
    serializable = [{field: record.get(field) for field in fields} for record in records]
    (output_dir / "all_trials_summary.json").write_text(
        json.dumps({"trials": serializable, "skipped": skipped}, indent=2),
        encoding="utf-8",
    )


def write_gallery(records: list[dict], skipped: list[dict], output_dir: Path, aggregate_images):
    def aggregate_title(path: Path):
        return (
            path.stem.replace("_", " ").title()
            .replace("Fl ", "FL ")
            .replace("Mpc", "MPC")
            .replace("Wbc", "WBC")
            .replace("Nominal", "Baseline")
        )

    def record_article(record):
        trial = record["trial"]
        status = "PASS" if record.get("success") else f"FAIL: {record.get('fall_reason')}"
        status_token = "pass" if record.get("success") else "fail"
        mode_token = str(record.get("robust", "unknown")).lower()
        images = []
        for filename in ("tracking_rmse.png", "robust_phase_foot_z.png", "mpc_timing.png"):
            if (record["trial_dir"] / filename).is_file():
                label = filename.removesuffix(".png").replace("_", " ")
                images.append(
                    f'<figure><figcaption>{html.escape(label)}</figcaption>'
                    f'<img loading="lazy" src="../{html.escape(trial)}/{filename}" '
                    f'alt="{html.escape(label)} for {html.escape(trial)}"></figure>'
                )
        return (
            f'<article class="trial" data-mode="{html.escape(mode_token)}" '
            f'data-status="{status_token}" data-search="{html.escape(trial.lower())}">'
            f'<h3>Robust {html.escape(str(record.get("robust")))}</h3>'
            f'<p><b>{html.escape(status)}</b> | offset={record.get("offset_m")} m | '
            f'trial={html.escape(trial)}</p>' + "".join(images) + "</article>"
        )

    monte_carlo_groups = defaultdict(dict)
    ordinary_records = []
    for record in records:
        match = MC_TAG_RE.search(record.get("trial", ""))
        if match:
            monte_carlo_groups[int(match.group(1))][match.group(3).upper()] = record
        else:
            ordinary_records.append(record)

    cards = []
    for sample_id, modes in sorted(monte_carlo_groups.items()):
        representative = modes.get("ON") or modes.get("OFF")
        offset = representative.get("offset_m")
        offset_label = "unknown" if offset is None else f"{float(offset):+.5f}"
        states = []
        for mode in ("ON", "OFF"):
            record = modes.get(mode)
            if record is None:
                states.append(f"{mode}: missing")
            else:
                states.append(f"{mode}: {'PASS' if record.get('success') else 'FAIL'}")
        articles = "".join(record_article(modes[mode]) for mode in ("ON", "OFF") if mode in modes)
        searchable = " ".join(record["trial"].lower() for record in modes.values())
        cards.append(
            f'<details class="sample" data-search="sample {sample_id:04d} {html.escape(searchable)}">'
            f'<summary><b>Sample {sample_id:04d}</b> · offset={offset_label} m · '
            f'{html.escape(" · ".join(states))}</summary>'
            f'<div class="pair-grid">{articles}</div></details>'
        )
    for record in ordinary_records:
        cards.append(f'<section>{record_article(record)}</section>')

    aggregate_html = "".join(
        f'<section><h2>{html.escape(aggregate_title(path))}</h2>'
        f'<a href="{html.escape(path.name)}"><img src="{html.escape(path.name)}" '
        f'alt="{html.escape(path.stem.replace("_", " "))}"></a></section>'
        for path in aggregate_images
    )
    skipped_html = "".join(
        f'<li>{html.escape(item["trial"])}: '
        f'{html.escape(item.get("reason") or item.get("error") or "unknown error")}</li>'
        for item in skipped
    ) or "<li>None</li>"
    selection_report = output_dir / "wbc_max_contrast_selection.json"
    selection_link = (
        '<a href="wbc_max_contrast_selection.json">Max-contrast selected trials (JSON)</a>'
        if selection_report.is_file()
        else ""
    )
    dashboard_table = output_dir / "all_trials_dashboard_comparison.csv"
    dashboard_table_link = (
        '<a href="all_trials_dashboard_comparison.csv">Dashboard comparison table (CSV)</a>'
        if dashboard_table.is_file()
        else ""
    )
    sweep_table_links = "".join(
        f'<a href="{html.escape(path.name)}">{html.escape(path.stem)} (CSV)</a>'
        for path in sorted(output_dir.glob("terrain_error_*.csv"))
    )
    monte_carlo_resources = (
        ("Fig. 1(a) overall success (PDF)", output_dir / "fig1a_overall_success.pdf"),
        ("Fig. 1(b) success vs dz (PDF)", output_dir / "fig1b_success_vs_dz.pdf"),
        ("Fig. 1(c) touchdown speed vs dz (PDF)", output_dir / "fig1c_touchdown_speed_vs_dz.pdf"),
        ("Fig. 1(d) base orientation RMSE vs dz (PDF)", output_dir / "fig1d_base_orientation_rmse_vs_dz.pdf"),
        ("Fig. 1(e) base position RMSE vs dz (PDF)", output_dir / "fig1e_base_position_rmse_vs_dz.pdf"),
        ("Table 1 (CSV)", output_dir / "table1_monte_carlo_results.csv"),
        ("Table 1 (LaTeX)", output_dir / "table1_monte_carlo_results.tex"),
        ("Table 1 (PDF)", output_dir / "table1_monte_carlo_results.pdf"),
        ("Fig. 2 (PDF)", output_dir / "fig2_computational_performance.pdf"),
        ("FL foothold extremes (PDF)", output_dir / "fl_foothold_extremes.pdf"),
        ("FL foothold selection (JSON)", output_dir / "fl_foothold_extremes_selection.json"),
        ("Monte Carlo paired-nearest FL foothold Robust ON (PDF)", output_dir / "fl_foothold_extremes_robust_on.pdf"),
        ("Monte Carlo paired-nearest FL foothold Robust ON selection (JSON)", output_dir / "fl_foothold_extremes_robust_on_selection.json"),
        ("Monte Carlo paired-nearest FL foothold Robust OFF (PDF)", output_dir / "fl_foothold_extremes_robust_off.pdf"),
        ("Monte Carlo paired-nearest FL foothold Robust OFF selection (JSON)", output_dir / "fl_foothold_extremes_robust_off_selection.json"),
        ("Fig. 3 paired FL touchdown window (PDF)", output_dir / "fig3_nominal_proposed_fl_touchdown_dz_m048_full.pdf"),
        ("Fig. 3 paired FL touchdown selection (JSON)", output_dir / "fig3_nominal_proposed_fl_touchdown_dz_m048_full_selection.json"),
        ("Fig. 5 Baseline body-z tracking (PDF)", output_dir / "fig5_nominal_body_z_vs_reference.pdf"),
        ("Fig. 5 Baseline body-z selection (JSON)", output_dir / "fig5_nominal_body_z_vs_reference_selection.json"),
        ("Fig. 6 Proposed body-z tracking (PDF)", output_dir / "fig6_proposed_body_z_vs_reference.pdf"),
        ("Fig. 6 Proposed body-z selection (JSON)", output_dir / "fig6_proposed_body_z_vs_reference_selection.json"),
        ("Fig. 7 Baseline body/foot overlay (PDF)", output_dir / "fig7_nominal_body_foot_z.pdf"),
        ("Fig. 7 Baseline body/foot selection (JSON)", output_dir / "fig7_nominal_body_foot_z_selection.json"),
        ("Fig. 8 Proposed body/foot overlay (PDF)", output_dir / "fig8_proposed_body_foot_z.pdf"),
        ("Fig. 8 Proposed body/foot selection (JSON)", output_dir / "fig8_proposed_body_foot_z_selection.json"),
        ("Fig. 9 full Monte Carlo velocity with 95% CI (PDF)", output_dir / "fig9_nominal_proposed_vcmd_vs_body_vx_full.pdf"),
        ("Fig. 9 Monte Carlo velocity aggregation (JSON)", output_dir / "fig9_nominal_proposed_vcmd_vs_body_vx_full_selection.json"),
        ("Fig. 10 successful base orientation RMSE (PDF)", output_dir / "fig10_base_orientation_rmse_successful.pdf"),
        ("Fig. 10 successful base orientation RMSE selection (JSON)", output_dir / "fig10_base_orientation_rmse_successful_selection.json"),
        ("Fig. 11 successful base position RMSE (PDF)", output_dir / "fig11_base_position_rmse_successful.pdf"),
        ("Fig. 11 successful base position RMSE selection (JSON)", output_dir / "fig11_base_position_rmse_successful_selection.json"),
        ("EXP matched-error FL foothold Robust ON (PDF)", output_dir / "fl_foothold_response_dz_p03_robust_on.pdf"),
        ("EXP matched-error FL foothold Robust ON selection (JSON)", output_dir / "fl_foothold_response_dz_p03_robust_on_selection.json"),
        ("EXP matched-error FL foothold Robust OFF (PDF)", output_dir / "fl_foothold_response_dz_p03_robust_off.pdf"),
        ("EXP matched-error FL foothold Robust OFF selection (JSON)", output_dir / "fl_foothold_response_dz_p03_robust_off_selection.json"),
        ("Monte Carlo summary (CSV)", output_dir / "monte_carlo_summary.csv"),
        ("Paired samples (CSV)", output_dir / "monte_carlo_pairs.csv"),
    )
    monte_carlo_table_links = "".join(
        f'<a href="{html.escape(path.name)}">{html.escape(label)}</a>'
        for label, path in monte_carlo_resources if path.is_file()
    )
    trial_controls = ""
    if monte_carlo_groups:
        trial_controls = """
<div class="filters">
  <label>Search <input id="trial-search" type="search" placeholder="sample ID or trial name"></label>
  <label>Mode <select id="mode-filter"><option value="all">All</option><option value="on">ON</option><option value="off">OFF</option></select></label>
  <label>Status <select id="status-filter"><option value="all">All</option><option value="pass">PASS</option><option value="fail">FAIL</option></select></label>
  <span id="visible-count" aria-live="polite"></span>
</div>"""
    trial_script = ""
    if monte_carlo_groups:
        trial_script = """
<script>
const search = document.getElementById('trial-search');
const mode = document.getElementById('mode-filter');
const status = document.getElementById('status-filter');
const count = document.getElementById('visible-count');
function applyTrialFilters() {
  const query = search.value.trim().toLowerCase();
  let visible = 0;
  document.querySelectorAll('details.sample').forEach(sample => {
    let sampleVisible = false;
    sample.querySelectorAll('article.trial').forEach(trial => {
      const matchesSearch = !query || sample.dataset.search.includes(query) || trial.dataset.search.includes(query);
      const matchesMode = mode.value === 'all' || trial.dataset.mode === mode.value;
      const matchesStatus = status.value === 'all' || trial.dataset.status === status.value;
      trial.hidden = !(matchesSearch && matchesMode && matchesStatus);
      sampleVisible ||= !trial.hidden;
    });
    sample.hidden = !sampleVisible;
    if (sampleVisible) visible += 1;
  });
  count.textContent = `${visible} samples shown`;
}
[search, mode, status].forEach(control => control.addEventListener('input', applyTrialFilters));
applyTrialFilters();
</script>"""
    page = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>GO2 experiment visualizations</title>
<style>
body{{font-family:sans-serif;margin:24px;background:#f5f6f8;color:#1c2430}}
section{{background:white;padding:16px;margin:18px 0;border-radius:8px;box-shadow:0 1px 5px #ccd}}
img{{max-width:100%;height:auto;margin:8px 0;border:1px solid #ddd}}
a{{margin-right:18px}} h1,h2,h3{{overflow-wrap:anywhere}}
details.sample{{background:white;margin:12px 0;border:1px solid #d8dde6;border-radius:8px}}
details.sample>summary{{cursor:pointer;padding:14px 16px}}
.pair-grid{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px;padding:0 16px 16px}}
.trial{{min-width:0;border-top:3px solid #d8dde6}}
.trial[data-mode="on"]{{border-top-color:#0072b2}}
.trial[data-mode="off"]{{border-top-color:#d55e00}}
figure{{margin:12px 0}} figcaption{{font-size:.9rem;color:#586174}}
.filters{{position:sticky;top:0;z-index:5;display:flex;gap:14px;align-items:end;flex-wrap:wrap;background:#f5f6f8;padding:12px 0}}
.filters label{{display:grid;gap:4px;font-weight:bold}} input,select{{padding:7px;border:1px solid #aeb5c1;border-radius:4px;background:white}}
[hidden]{{display:none!important}}
@media(max-width:850px){{.pair-grid{{grid-template-columns:1fr}}body{{margin:12px}}}}
</style></head><body>
<h1>GO2 perceptive robust-phase experiment gallery</h1>
<p>{len(records)} valid trials; {len(skipped)} skipped.</p>
<p><a href="all_trials_summary.csv">CSV summary</a>
<a href="all_trials_summary.json">JSON summary</a>
{dashboard_table_link}
{sweep_table_links}
{monte_carlo_table_links}
{selection_link}</p>
{aggregate_html}
<h2>Skipped/incomplete trials</h2><ul>{skipped_html}</ul>
<h1>Per-trial plots</h1>{trial_controls}{''.join(cards)}
{trial_script}
</body></html>"""
    (output_dir / "index.html").write_text(page, encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--skip-individual", action="store_true")
    parser.add_argument(
        "--force-individual",
        action="store_true",
        help="Regenerate per-trial plots even when complete outputs are newer than their inputs.",
    )
    parser.add_argument("--skip-foot-z", action="store_true")
    parser.add_argument("--skip-wbc", action="store_true", help="Skip aggregate WBC plots.")
    args = parser.parse_args(argv)
    requested_results_name = args.results_dir.name
    results_dir = args.results_dir.resolve()
    output_dir = results_dir / "all_visualizations"
    output_dir.mkdir(parents=True, exist_ok=True)
    monte_carlo_manifest = results_dir / "samples.csv"
    monte_carlo = monte_carlo_manifest.is_file()
    records, skipped = load_trials(results_dir)
    if not records:
        raise RuntimeError(f"No valid trials found in {results_dir}")
    metric_failures = attach_trial_metrics(records)
    attach_mpc_timing(records)
    robust_states = {record.get("robust") for record in records}
    terrain_offsets = {
        float(record["offset_m"])
        for record in records
        if record.get("offset_m") is not None
    }
    terrain_sweep = robust_states == {"ON"} and (
        "terrain_error_sweep" in requested_results_name
        or "terrain_error_sweep" in results_dir.name
        or len(terrain_offsets) >= 2
    )

    failures = list(metric_failures)
    if not args.skip_individual:
        reused_products = 0
        regenerated_products = 0
        fully_cached_trials = 0
        for index, record in enumerate(records, 1):
            trial_dir = record["trial_dir"]
            tasks = [("plot_trial_rmse.py", "tracking_rmse")]
            if not args.skip_foot_z:
                tasks.append(("plot_robust_phase.py", "robust_phase_foot_z"))
            if record.get("mpc_solve_ms_avg") is not None:
                tasks.append(("plot_mpc_timing.py", "mpc_timing"))

            pending = []
            for script_name, plot_name in tasks:
                if (
                    not args.force_individual
                    and individual_plot_is_current(script_name, trial_dir)
                ):
                    reused_products += 1
                else:
                    pending.append((script_name, plot_name))
            if not pending:
                fully_cached_trials += 1
                continue

            print(f"[{index}/{len(records)}] {trial_dir.name}", flush=True)
            for script_name, plot_name in pending:
                ok, detail = run_plot(ROOT / script_name, trial_dir)
                if not ok:
                    failures.append(
                        {"trial": trial_dir.name, "plot": plot_name, "error": detail}
                    )
                else:
                    regenerated_products += 1
        print(
            "individual plots: "
            f"{reused_products} cached products reused, "
            f"{regenerated_products} regenerated, "
            f"{fully_cached_trials}/{len(records)} trials fully cached"
        )

    attach_rmse(records)
    attach_terrain_descent(records)
    if monte_carlo:
        # Exact offsets are intentionally unique in a Monte Carlo design, so
        # the categorical dashboards would produce one zero-variance bar per
        # trial. Replace them with continuous-response and paired dashboards.
        for stale_name in (
            "all_trials_dashboard.png",
            "all_trials_mpc_timing.png",
            "all_trials_robust.png",
            "all_trials_dashboard_comparison.csv",
        ):
            stale_path = output_dir / stale_name
            if stale_path.is_file():
                stale_path.unlink()
        try:
            aggregate_images = generate_monte_carlo_outputs(
                records, monte_carlo_manifest, output_dir
            )
        except (OSError, RuntimeError, ValueError) as exc:
            failures.append(
                {"trial": "aggregate", "plot": "monte_carlo_dashboard", "error": str(exc)}
            )
            aggregate_images = []
    else:
        dashboard = output_dir / "all_trials_dashboard.png"
        plot_dashboard(records, dashboard)
        mpc_dashboard = output_dir / "all_trials_mpc_timing.png"
        has_mpc_timing = any(record.get("mpc_solve_ms_avg") is not None for record in records)
        if has_mpc_timing:
            plot_mpc_dashboard(records, mpc_dashboard)
        elif mpc_dashboard.is_file():
            mpc_dashboard.unlink()
        dashboard_comparison = output_dir / "all_trials_dashboard_comparison.csv"
        if terrain_sweep:
            if dashboard_comparison.is_file():
                dashboard_comparison.unlink()
        else:
            write_dashboard_comparison_table(records, dashboard_comparison)
        robust_dashboard = output_dir / "all_trials_robust.png"
        has_robust_metrics = plot_robust_dashboard(records, robust_dashboard)
        if not has_robust_metrics and robust_dashboard.is_file():
            robust_dashboard.unlink()

        aggregate_images = [dashboard]
        if has_mpc_timing:
            aggregate_images.append(mpc_dashboard)
        if has_robust_metrics:
            aggregate_images.append(robust_dashboard)
    write_summary(records, skipped + failures, output_dir)

    if monte_carlo:
        # Old aggregate plots grouped rounded offsets into dozens of cohorts;
        # keep the Monte Carlo gallery focused on the paired summaries.
        for pattern in (
            "wbc_tracking_error*.png",
            "wbc_deviation_components*.png",
            "terrain_error_*.png",
            "terrain_error_*.csv",
        ):
            for stale_output in output_dir.glob(pattern):
                stale_output.unlink()
        selection_report = output_dir / "wbc_max_contrast_selection.json"
        if selection_report.is_file():
            selection_report.unlink()
    elif terrain_sweep:
        for pattern in ("wbc_tracking_error*.png", "wbc_deviation_components*.png"):
            for stale_output in output_dir.glob(pattern):
                stale_output.unlink()
        selection_report = output_dir / "wbc_max_contrast_selection.json"
        if selection_report.is_file():
            selection_report.unlink()
        for pattern in ("terrain_error_*.png", "terrain_error_*.csv"):
            for stale_output in output_dir.glob(pattern):
                stale_output.unlink()
        command = [
            sys.executable,
            str(ROOT / "plot_terrain_error_sweep.py"),
            "--results-dir",
            str(results_dir),
            "--out-dir",
            str(output_dir),
        ]
        if args.skip_wbc:
            command.append("--skip-wbc")
        proc = subprocess.run(command, capture_output=True, text=True, timeout=480.0)
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "plot_terrain_error_sweep.py",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
        aggregate_images.extend(sorted(output_dir.glob("terrain_error_*.png")))
    elif not args.skip_wbc:
        # WBC aggregate images are fully reproducible. Remove old naming schemes
        # and stale cohort outputs before generating the current complete set.
        for pattern in ("wbc_tracking_error*.png", "wbc_deviation_components*.png"):
            for stale_image in output_dir.glob(pattern):
                stale_image.unlink()
        selection_report = output_dir / "wbc_max_contrast_selection.json"
        if selection_report.is_file():
            selection_report.unlink()
        for script_name in ("plot_wbc_tracking_error.py", "plot_wbc_deviation_components.py"):
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / script_name),
                    "--results-dir",
                    str(results_dir),
                    "--out-dir",
                    str(output_dir),
                ],
                capture_output=True,
                text=True,
                timeout=240.0,
            )
            if proc.returncode != 0:
                failures.append(
                    {"trial": "aggregate", "plot": script_name, "error": proc.stderr.strip()}
                )
        aggregate_images.extend(sorted(output_dir.glob("wbc_tracking_error_*.png")))
        aggregate_images.extend(sorted(output_dir.glob("wbc_deviation_components_*.png")))

    if monte_carlo and not args.skip_wbc:
        rmse_outputs = (
            output_dir / "fig10_base_orientation_rmse_successful.png",
            output_dir / "fig11_base_position_rmse_successful.png",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "plot_wbc_successful_rmse_paper.py"),
                "--results-dir",
                str(results_dir),
                "--out-dir",
                str(output_dir),
            ],
            capture_output=True,
            text=True,
            timeout=240.0,
        )
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "Successful-trial base RMSE figures",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
            for rmse_output in rmse_outputs:
                for stale_output in (
                    rmse_output,
                    rmse_output.with_suffix(".pdf"),
                    rmse_output.with_name(f"{rmse_output.stem}_selection.json"),
                ):
                    if stale_output.is_file():
                        stale_output.unlink()
        else:
            aggregate_images.extend(output for output in rmse_outputs if output.is_file())

    # A fixed terrain sweep can show the exact +/-0.05 m Robust-ON trials.
    # Monte Carlo is handled separately below so its ON/OFF comparison never
    # mixes sources, sample IDs, seeds, or terrain offsets.
    if terrain_sweep and not args.skip_foot_z:
        foothold_output = output_dir / "fl_foothold_extremes.png"
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "plot_fl_foothold_extremes.py"),
                "--results-dir",
                str(results_dir),
                "--out-dir",
                str(output_dir),
            ],
            capture_output=True,
            text=True,
            timeout=240.0,
        )
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "plot_fl_foothold_extremes.py",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
            if foothold_output.is_file():
                foothold_output.unlink()
        elif foothold_output.is_file():
            aggregate_images.append(foothold_output)

    # Use the same paired Monte Carlo samples for ON and OFF. Selection ignores
    # outcome and chooses the sampled dz nearest each +/-0.05 m bound. Both
    # figures use identical x/y limits for direct comparison.
    if monte_carlo and not args.skip_foot_z:
        for stale_name in (
            "fl_foothold_extremes.png",
            "fl_foothold_extremes.pdf",
            "fl_foothold_extremes_selection.json",
            "fl_foothold_extremes_monte_carlo_robust_on_any_outcome.png",
            "fl_foothold_extremes_monte_carlo_robust_on_any_outcome.pdf",
            "fl_foothold_extremes_monte_carlo_robust_on_any_outcome_selection.json",
        ):
            stale_output = output_dir / stale_name
            if stale_output.is_file():
                stale_output.unlink()

        for mode in ("on", "off"):
            foothold_output = output_dir / f"fl_foothold_extremes_robust_{mode}.png"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "plot_fl_foothold_extremes.py"),
                    "--results-dir",
                    str(results_dir),
                    "--out-dir",
                    str(output_dir),
                    "--robust",
                    mode,
                    "--allow-nearest",
                    "--any-outcome",
                    "--output-name",
                    foothold_output.name,
                    "--y-limits",
                    "0.045",
                    "0.375",
                ],
                capture_output=True,
                text=True,
                timeout=240.0,
            )
            if proc.returncode != 0:
                failures.append(
                    {
                        "trial": "aggregate",
                        "plot": f"Monte Carlo paired-nearest FL foothold Robust {mode.upper()}",
                        "error": proc.stderr.strip() or proc.stdout.strip(),
                    }
                )
                for stale_output in (
                    foothold_output,
                    foothold_output.with_suffix(".pdf"),
                    foothold_output.with_name(f"{foothold_output.stem}_selection.json"),
                ):
                    if stale_output.is_file():
                        stale_output.unlink()
            elif foothold_output.is_file():
                aggregate_images.append(foothold_output)

        paper_foothold_output = output_dir / "fig3_nominal_proposed_fl_touchdown_dz_m048_full.png"
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "plot_paired_foothold_paper.py"),
                "--results-dir",
                str(results_dir),
                "--out-dir",
                str(output_dir),
                "--time-view",
                "full",
                "--output-name",
                paper_foothold_output.name,
            ],
            capture_output=True,
            text=True,
            timeout=240.0,
        )
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "Combined Baseline/Proposed FL touchdown figure",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
            for stale_output in (
                paper_foothold_output,
                paper_foothold_output.with_suffix(".pdf"),
                paper_foothold_output.with_name(
                    f"{paper_foothold_output.stem}_selection.json"
                ),
            ):
                if stale_output.is_file():
                    stale_output.unlink()
        else:
            if paper_foothold_output.is_file():
                aggregate_images.append(paper_foothold_output)

        body_outputs = (
            output_dir / "fig5_nominal_body_z_vs_reference.png",
            output_dir / "fig6_proposed_body_z_vs_reference.png",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "plot_paired_body_z_paper.py"),
                "--results-dir",
                str(results_dir),
                "--out-dir",
                str(output_dir),
            ],
            capture_output=True,
            text=True,
            timeout=240.0,
        )
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "Independent Baseline/Proposed body-z tracking figures",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
            for body_output in body_outputs:
                for stale_output in (
                    body_output,
                    body_output.with_suffix(".pdf"),
                    body_output.with_name(f"{body_output.stem}_selection.json"),
                ):
                    if stale_output.is_file():
                        stale_output.unlink()
        else:
            aggregate_images.extend(
                body_output for body_output in body_outputs if body_output.is_file()
            )

        combined_outputs = (
            output_dir / "fig7_nominal_body_foot_z.png",
            output_dir / "fig8_proposed_body_foot_z.png",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "plot_paired_body_foot_paper.py"),
                "--results-dir",
                str(results_dir),
                "--out-dir",
                str(output_dir),
            ],
            capture_output=True,
            text=True,
            timeout=240.0,
        )
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "Independent Baseline/Proposed body-foot overlay figures",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
            for combined_output in combined_outputs:
                for stale_output in (
                    combined_output,
                    combined_output.with_suffix(".pdf"),
                    combined_output.with_name(
                        f"{combined_output.stem}_selection.json"
                    ),
                ):
                    if stale_output.is_file():
                        stale_output.unlink()
        else:
            aggregate_images.extend(
                combined_output
                for combined_output in combined_outputs
                if combined_output.is_file()
            )

        velocity_outputs = (
            output_dir / "fig9_nominal_proposed_vcmd_vs_body_vx_full.png",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "plot_paired_velocity_paper.py"),
                "--results-dir",
                str(results_dir),
                "--out-dir",
                str(output_dir),
            ],
            capture_output=True,
            text=True,
            timeout=240.0,
        )
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "Full-duration Monte Carlo velocity CI figure",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
            for velocity_output in velocity_outputs:
                for stale_output in (
                    velocity_output,
                    velocity_output.with_suffix(".pdf"),
                    velocity_output.with_name(
                        f"{velocity_output.stem}_selection.json"
                    ),
                ):
                    if stale_output.is_file():
                        stale_output.unlink()
        else:
            aggregate_images.extend(
                velocity_output
                for velocity_output in velocity_outputs
                if velocity_output.is_file()
            )

    # The fixed-error EXP dataset contains repeated ON/OFF trials at exactly
    # +/-0.03 m. Show one successful, temporally adjacent pair at the same
    # +0.03 m error and run number, using identical axes for direct comparison.
    if results_dir.name == "exp" and not args.skip_foot_z:
        exp_pair = select_exp_foothold_pair(records)
        if not exp_pair:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "matched-error FL foothold pair",
                    "error": "No adjacent successful Robust ON/OFF pair at dz=+0.03 m, run 1",
                }
            )
        for mode in ("ON", "OFF"):
            if mode not in exp_pair:
                continue
            mode_slug = mode.lower()
            foothold_output = output_dir / f"fl_foothold_response_dz_p03_robust_{mode_slug}.png"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "plot_fl_foothold_extremes.py"),
                    "--trial-dir",
                    str(exp_pair[mode]["trial_dir"]),
                    "--out-dir",
                    str(output_dir),
                    "--output-name",
                    foothold_output.name,
                    "--y-limits",
                    "0.13",
                    "0.385",
                ],
                capture_output=True,
                text=True,
                timeout=240.0,
            )
            if proc.returncode != 0:
                failures.append(
                    {
                        "trial": exp_pair[mode]["trial"],
                        "plot": f"matched-error FL foothold Robust {mode}",
                        "error": proc.stderr.strip() or proc.stdout.strip(),
                    }
                )
                for stale_output in (
                    foothold_output,
                    foothold_output.with_suffix(".pdf"),
                    foothold_output.with_name(f"{foothold_output.stem}_selection.json"),
                ):
                    if stale_output.is_file():
                        stale_output.unlink()
            elif foothold_output.is_file():
                aggregate_images.append(foothold_output)

    # Rewrite after aggregate generation so late failures are included.
    write_summary(records, skipped + failures, output_dir)
    write_gallery(records, skipped + failures, output_dir, aggregate_images)
    print(f"saved gallery: {output_dir / 'index.html'}")
    print(f"valid trials: {len(records)}, skipped/plot failures: {len(skipped) + len(failures)}")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
