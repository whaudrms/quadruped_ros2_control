#!/usr/bin/env python3
"""Generate continuous-response and paired dashboards for Monte Carlo trials."""

import csv
import math
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from paper_plot_style import (
    FAILURE_COLOR as PAPER_FAILURE_COLOR,
    LINE_WIDTH as PAPER_LINE_WIDTH,
    MARKER_SIZE as PAPER_MARKER_SIZE,
    NOMINAL_COLOR as PAPER_NOMINAL_COLOR,
    NOMINAL_MARKER,
    PROPOSED_COLOR as PAPER_PROPOSED_COLOR,
    PROPOSED_MARKER,
    apply_paper_style,
    paper_legend,
    style_paper_axis,
)


ON_COLOR = "#0072B2"
OFF_COLOR = "#D55E00"
FAIL_COLOR = "#B2182B"
GRID_COLOR = "#B8BDC7"
MC_TAG_RE = re.compile(r"(?:^|_)mc_s(\d+)_seed(\d+)_(on|off)(?:_|$)", re.I)

PAIRED_METRICS = (
    ("success", "Success", "percentage points", True),
    ("body_frame_forward_progress", "Forward progress", "m", True),
    ("base_position_rmse_m", "Base position RMSE", "m", False),
    ("base_orientation_rmse_deg", "Base orientation RMSE", "deg", False),
    ("joint_position_rmse_rad", "Joint position RMSE", "rad", False),
    ("touchdown_normal_speed_mean_mps", "Touchdown normal speed", "m/s", False),
)

TIMING_METRICS = (
    ("mpc_solve_ms_avg", "Mean MPC solve time", "ms"),
    ("mpc_solve_ms_p95", "P95 MPC solve time", "ms"),
    ("wbc_solve_ms_mean", "Mean WBC compute time", "ms"),
    ("wbc_solve_ms_p95", "P95 WBC compute time", "ms"),
)

PAPER_TABLE_METRICS = (
    ("success", "Success", "%", True),
    ("body_frame_forward_progress", "Forward progress", "m", True),
    ("body_frame_lateral_progress", "Lateral drift", "m", False),
    ("base_position_rmse_m", "Base position RMSE", "m", False),
    ("base_orientation_rmse_deg", "Base orientation RMSE", "deg", False),
    ("joint_position_rmse_rad", "Joint RMSE", "rad", False),
    ("touchdown_normal_speed_mean_mps", "Touchdown normal speed", "m/s", False),
)


def _finite(value):
    if value is None:
        return None
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted if np.isfinite(converted) else None


def _metric_value(record: dict | None, metric: str):
    if record is None:
        return None
    if metric == "success":
        return 100.0 if bool(record.get("success")) else 0.0
    value = _finite(record.get(metric))
    if value is not None and metric == "body_frame_lateral_progress":
        value = abs(value)
    return value


def _load_manifest(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    samples = []
    for row in rows:
        samples.append(
            {
                "sample_id": int(row["sample_id"]),
                "sample_seed": int(row["sample_seed"]),
                "offset_m": float(row["terrain_z_offset"]),
                "first_mode": row["first_mode"].upper(),
            }
        )
    return samples


def _pair_trials(records: list[dict], samples: list[dict]) -> list[dict]:
    indexed = {}
    for record in records:
        match = MC_TAG_RE.search(record.get("trial", ""))
        if not match:
            continue
        sample_id = int(match.group(1))
        seed = int(match.group(2))
        mode = match.group(3).upper()
        indexed[(sample_id, seed, mode)] = record

    pairs = []
    for sample in samples:
        sample_id = sample["sample_id"]
        seed = sample["sample_seed"]
        pairs.append(
            {
                **sample,
                "ON": indexed.get((sample_id, seed, "ON")),
                "OFF": indexed.get((sample_id, seed, "OFF")),
            }
        )
    return pairs


def _wilson(successes: int, total: int, z: float = 1.96):
    if total <= 0:
        return np.nan, np.nan
    probability = successes / total
    denominator = 1.0 + z * z / total
    center = (probability + z * z / (2.0 * total)) / denominator
    half = z * math.sqrt(
        probability * (1.0 - probability) / total + z * z / (4.0 * total * total)
    ) / denominator
    return 100.0 * (center - half), 100.0 * (center + half)


def _bootstrap_mean_ci(values, rng, samples: int = 4000):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return np.nan, np.nan
    if values.size == 1:
        return float(values[0]), float(values[0])
    indices = rng.integers(0, values.size, size=(samples, values.size))
    means = np.mean(values[indices], axis=1)
    return tuple(float(value) for value in np.percentile(means, [2.5, 97.5]))


def _metric_bootstrap_ci(values, metric: str):
    """Return a stable metric-specific CI shared by figures and tables."""
    metric_seed = 20260831 + sum(
        (index + 1) * ord(character) for index, character in enumerate(metric)
    )
    return _bootstrap_mean_ci(values, np.random.default_rng(metric_seed))


def _bin_series(x, y, edges, *, success=False):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    indices = np.clip(np.digitize(x, edges[1:-1], right=False), 0, len(edges) - 2)
    centers, means, lower, upper, counts = [], [], [], [], []
    for index in range(len(edges) - 1):
        values = y[indices == index]
        if values.size == 0:
            continue
        centers.append((edges[index] + edges[index + 1]) / 2.0)
        mean = float(np.mean(values))
        means.append(mean)
        counts.append(int(values.size))
        if success:
            lo, hi = _wilson(int(np.count_nonzero(values > 50.0)), int(values.size))
        elif values.size > 1:
            half = 1.96 * float(np.std(values, ddof=1)) / math.sqrt(values.size)
            lo, hi = mean - half, mean + half
        else:
            lo = hi = mean
        lower.append(lo)
        upper.append(hi)
    return tuple(np.asarray(values) for values in (centers, means, lower, upper, counts))


def _style_axis(axis, *, zero_x=True):
    axis.grid(axis="y", color=GRID_COLOR, alpha=0.38, linewidth=0.8)
    axis.spines[["top", "right"]].set_visible(False)
    if zero_x:
        axis.axvline(0.0, color="0.45", linestyle="--", linewidth=1.0, zorder=0)


def _plot_response(axis, pairs, metric, title, unit, edges, *, success=False):
    any_values = False
    for mode, color in (("ON", ON_COLOR), ("OFF", OFF_COLOR)):
        x_values, y_values, failed = [], [], []
        for pair in pairs:
            record = pair[mode]
            value = _metric_value(record, metric)
            if value is None:
                continue
            x_values.append(100.0 * pair["offset_m"])
            y_values.append(value)
            failed.append(not bool(record.get("success")))
        if not x_values:
            continue
        any_values = True
        x_array = np.asarray(x_values)
        y_array = np.asarray(y_values)
        failed_array = np.asarray(failed)
        axis.scatter(
            x_array[~failed_array], y_array[~failed_array], color=color,
            alpha=0.30, s=22, edgecolors="none", zorder=2,
        )
        if np.any(failed_array):
            axis.scatter(
                x_array[failed_array], y_array[failed_array], color=FAIL_COLOR,
                alpha=0.75, marker="x", s=34, linewidth=1.2, zorder=3,
            )
        centers, means, lower, upper, counts = _bin_series(
            x_array, y_array, edges, success=success
        )
        if centers.size:
            axis.plot(centers, means, color=color, marker="o", linewidth=2.0,
                      markersize=4.5, label=f"Robust {mode}", zorder=4)
            axis.fill_between(centers, lower, upper, color=color, alpha=0.13, zorder=1)
            for x_value, y_value, count in zip(centers, means, counts):
                axis.annotate(
                    f"n={int(count)}", (x_value, y_value), xytext=(0, 7),
                    textcoords="offset points", ha="center", fontsize=7, color=color,
                )
    axis.set_title(title)
    axis.set_xlabel("Terrain perception error [cm]")
    axis.set_ylabel(unit)
    _style_axis(axis)
    if success:
        axis.set_ylim(-5.0, 105.0)
    if any_values:
        axis.legend(frameon=False, fontsize=9)
    else:
        axis.text(0.5, 0.5, "Metric unavailable", transform=axis.transAxes,
                  ha="center", va="center", color="0.4")


def _plot_overview(pairs: list[dict], output: Path, edges):
    completed = [pair for pair in pairs if pair["ON"] is not None and pair["OFF"] is not None]
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), squeeze=False)

    axis = axes[0, 0]
    for index, (mode, color) in enumerate((("ON", ON_COLOR), ("OFF", OFF_COLOR))):
        records = [pair[mode] for pair in pairs if pair[mode] is not None]
        successes = sum(bool(record.get("success")) for record in records)
        total = len(records)
        rate = 100.0 * successes / total if total else np.nan
        lo, hi = _wilson(successes, total)
        axis.bar(index, rate, color=color, alpha=0.82, width=0.58)
        if total:
            axis.errorbar(index, rate, yerr=[[rate - lo], [hi - rate]], fmt="none",
                          color="black", capsize=6, linewidth=1.3)
            axis.text(index, min(104.0, rate + 5.0), f"{successes}/{total}\n{rate:.1f}%",
                      ha="center", va="bottom", fontweight="bold")
    axis.set_title("Overall success with Wilson 95% CI")
    axis.set_ylabel("Success [%]")
    axis.set_xticks([0, 1], ["Robust ON", "Robust OFF"])
    axis.set_ylim(0.0, 115.0)
    _style_axis(axis, zero_x=False)

    axis = axes[0, 1]
    outcome_labels = ["Both pass", "ON only", "OFF only", "Both fail"]
    outcome_counts = [0, 0, 0, 0]
    for pair in completed:
        on_success = bool(pair["ON"].get("success"))
        off_success = bool(pair["OFF"].get("success"))
        if on_success and off_success:
            outcome_counts[0] += 1
        elif on_success:
            outcome_counts[1] += 1
        elif off_success:
            outcome_counts[2] += 1
        else:
            outcome_counts[3] += 1
    colors = ["#5B8E7D", ON_COLOR, OFF_COLOR, FAIL_COLOR]
    bars = axis.barh(outcome_labels[::-1], outcome_counts[::-1], color=colors[::-1], alpha=0.86)
    for bar, value in zip(bars, outcome_counts[::-1]):
        axis.text(bar.get_width() + 0.3, bar.get_y() + bar.get_height() / 2,
                  str(value), va="center", fontweight="bold")
    axis.set_title("Paired success outcomes")
    axis.set_xlabel("Number of paired samples")
    axis.set_xlim(0, max(outcome_counts + [1]) * 1.18)
    _style_axis(axis, zero_x=False)

    _plot_response(
        axes[1, 0], pairs, "success", "Success response by offset", "Success [%]",
        edges, success=True,
    )
    _plot_response(
        axes[1, 1], pairs, "body_frame_forward_progress",
        "Forward progress response", "Progress [m]", edges,
    )

    fig.suptitle(
        f"Monte Carlo overview — {len(completed)}/{len(pairs)} complete ON/OFF pairs\n"
        "points: trials; lines: offset-bin means; bands: 95% intervals; red ×: failed trial",
        fontsize=15, fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(output, dpi=160)
    plt.close(fig)


def _paired_improvements(pairs, metric, higher_is_better):
    rows = []
    for pair in pairs:
        on_value = _metric_value(pair["ON"], metric)
        off_value = _metric_value(pair["OFF"], metric)
        if on_value is None or off_value is None:
            continue
        improvement = on_value - off_value if higher_is_better else off_value - on_value
        rows.append((100.0 * pair["offset_m"], on_value, off_value, improvement))
    return rows


def _paper_response_panel(
    axis, pairs, metric, unit, edges, *, success=False, scatter=False, show_ci=False
):
    """Plot Baseline/Proposed response curves using paper-facing labels."""
    rng = np.random.default_rng(29)
    trial_marker_size = 28
    failure_marker_size = 22
    for mode, label, color, marker in (
        ("OFF", "Baseline", PAPER_NOMINAL_COLOR, NOMINAL_MARKER),
        ("ON", "Proposed", PAPER_PROPOSED_COLOR, PROPOSED_MARKER),
    ):
        x_values, y_values, failures = [], [], []
        for pair in pairs:
            record = pair[mode]
            value = _metric_value(record, metric)
            if value is None:
                continue
            x_values.append(100.0 * pair["offset_m"])
            y_values.append(value)
            failures.append(not bool(record.get("success")))
        if not x_values:
            continue
        x_array = np.asarray(x_values, dtype=float)
        y_array = np.asarray(y_values, dtype=float)
        failure_array = np.asarray(failures, dtype=bool)
        if scatter:
            jitter = rng.uniform(-0.045, 0.045, size=x_array.size)
            axis.scatter(
                x_array[~failure_array] + jitter[~failure_array],
                y_array[~failure_array],
                color=color, marker=marker, alpha=0.35, s=trial_marker_size,
                edgecolors="none", zorder=2,
            )
            if np.any(failure_array):
                axis.scatter(
                    x_array[failure_array] + jitter[failure_array],
                    y_array[failure_array],
                    facecolors="none", edgecolors=PAPER_FAILURE_COLOR,
                    marker=marker, alpha=0.85, s=failure_marker_size,
                    linewidth=1.0, zorder=3,
                )
        centers, means, lower, upper, _counts = _bin_series(
            x_array, y_array, edges, success=success
        )
        axis.plot(
            centers, means, color=color, marker=marker,
            markerfacecolor=color, markeredgecolor=color,
            markersize=PAPER_MARKER_SIZE, linewidth=PAPER_LINE_WIDTH,
            label=label, zorder=4,
        )
        if show_ci:
            axis.fill_between(
                centers, lower, upper, color=color, alpha=0.18, zorder=1
            )
    axis.set_xlabel(r"Terrain height error $\Delta z$ [cm]")
    axis.set_ylabel(unit)
    paper_legend(axis, loc="best")
    style_paper_axis(axis, minor=True)
    axis.axvline(0.0, color="0.35", linestyle="--", linewidth=1.0, zorder=0)
    if success:
        axis.set_ylim(0.0, 105.0)


def _save_png_pdf(fig, png_path: Path, *, dpi=300, preserve_canvas=False):
    bbox_inches = None if preserve_canvas else "tight"
    fig.savefig(png_path, dpi=dpi, bbox_inches=bbox_inches)
    fig.savefig(png_path.with_suffix(".pdf"), bbox_inches=bbox_inches)


def _plot_paper_figure1(pairs: list[dict], output_dir: Path, edges) -> list[Path]:
    outputs = [
        output_dir / "fig1a_overall_success.png",
        output_dir / "fig1b_success_vs_dz.png",
        output_dir / "fig1c_touchdown_speed_vs_dz.png",
        output_dir / "fig1d_base_orientation_rmse_vs_dz.png",
        output_dir / "fig1e_base_position_rmse_vs_dz.png",
    ]
    with plt.rc_context():
        apply_paper_style()

        fig, axis = plt.subplots(figsize=(8.0, 4.5))
        for index, (mode, color) in enumerate(
            (("OFF", PAPER_NOMINAL_COLOR), ("ON", PAPER_PROPOSED_COLOR))
        ):
            records = [pair[mode] for pair in pairs if pair[mode] is not None]
            successes = sum(bool(record.get("success")) for record in records)
            total = len(records)
            rate = 100.0 * successes / total if total else np.nan
            lo, hi = _wilson(successes, total)
            axis.bar(
                index, rate, color=color, edgecolor="0.20",
                linewidth=0.8, width=0.62,
            )
            if total:
                axis.errorbar(
                    index, rate, yerr=[[rate - lo], [hi - rate]], fmt="none",
                    color="0.15", capsize=6, capthick=1.1, linewidth=1.1,
                )
                axis.text(
                    index, min(108.0, hi + 2.0),
                    f"{rate:.0f}%\n({successes}/{total})",
                    ha="center", va="bottom", fontsize=13,
                )
        axis.set_ylabel("Success rate [%]")
        axis.set_xticks([0, 1], ["Baseline", "Proposed"])
        axis.set_ylim(0.0, 115.0)
        style_paper_axis(axis, grid_axis="y")
        fig.tight_layout(pad=0.35)
        _save_png_pdf(fig, outputs[0], preserve_canvas=True)
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(8.0, 4.5))
        _paper_response_panel(
            axis, pairs, "success", "Success rate [%]",
            edges, success=True, scatter=False,
        )
        fig.tight_layout(pad=0.35)
        _save_png_pdf(fig, outputs[1], preserve_canvas=True)
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(8.0, 4.5))
        _paper_response_panel(
            axis, pairs, "touchdown_normal_speed_mean_mps",
            r"Touchdown normal speed [m/s]", edges, scatter=True, show_ci=True,
        )
        fig.tight_layout(pad=0.35)
        _save_png_pdf(fig, outputs[2], preserve_canvas=True)
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(8.0, 4.5))
        _paper_response_panel(
            axis, pairs, "base_orientation_rmse_deg",
            r"Base orientation RMSE [deg]", edges, scatter=True, show_ci=True,
        )
        fig.tight_layout(pad=0.35)
        _save_png_pdf(fig, outputs[3], preserve_canvas=True)
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(8.0, 4.5))
        _paper_response_panel(
            axis, pairs, "base_position_rmse_m",
            r"Base position RMSE [m]", edges, scatter=True, show_ci=True,
        )
        fig.tight_layout(pad=0.35)
        _save_png_pdf(fig, outputs[4], preserve_canvas=True)
        plt.close(fig)
    return outputs


def _boxplot_with_points(axis, off_values, on_values, *, seed=31):
    box = axis.boxplot(
        [off_values, on_values], labels=["Baseline", "Proposed"], widths=0.54,
        patch_artist=True, showmeans=True, showfliers=False,
        medianprops={"color": "0.15", "linewidth": 1.4},
        whiskerprops={"color": "0.30", "linestyle": "--", "linewidth": 1.0},
        capprops={"color": "0.30", "linewidth": 1.0},
        meanprops={"marker": "D", "markerfacecolor": "white",
                   "markeredgecolor": "0.15", "markersize": 6},
    )
    for patch, color in zip(box["boxes"], (PAPER_NOMINAL_COLOR, PAPER_PROPOSED_COLOR)):
        patch.set_facecolor(color)
        patch.set_alpha(0.66)
        patch.set_edgecolor("0.20")
        patch.set_linewidth(0.8)
    rng = np.random.default_rng(seed)
    series = (
        (off_values, PAPER_NOMINAL_COLOR, NOMINAL_MARKER),
        (on_values, PAPER_PROPOSED_COLOR, PROPOSED_MARKER),
    )
    for index, (values, color, marker) in enumerate(series, 1):
        x_values = index + rng.uniform(-0.10, 0.10, size=len(values))
        axis.scatter(
            x_values, values, color=color, marker=marker, alpha=0.32, s=25,
            edgecolors="none", zorder=1,
        )
    style_paper_axis(axis, minor=True, grid_axis="y")


def _plot_paper_figure2(pairs: list[dict], output: Path):
    with plt.rc_context():
        apply_paper_style()
        fig, axes = plt.subplots(1, 2, figsize=(12.0, 6.75), squeeze=False)
        axes = axes[0]
        metric_specs = (
            ("mpc_solve_ms_avg", "Mean MPC solve time [ms]"),
            ("mpc_solve_ms_p95", "P95 MPC solve time [ms]"),
        )
        for index, (axis, (metric, ylabel)) in enumerate(zip(axes, metric_specs)):
            rows = _paired_improvements(pairs, metric, higher_is_better=False)
            if not rows:
                axis.text(0.5, 0.5, "Metric unavailable", transform=axis.transAxes,
                          ha="center", va="center")
                axis.set_axis_off()
                continue
            values = np.asarray(rows, dtype=float)
            off_values = values[:, 2]
            on_values = values[:, 1]
            reductions = values[:, 3]
            lo, hi = _metric_bootstrap_ci(reductions, metric)
            _boxplot_with_points(axis, off_values, on_values, seed=41 + index)
            axis.set_ylabel(ylabel)
            annotation = (
                f"Baseline mean: {np.mean(off_values):.2f} ms\n"
                f"Proposed mean: {np.mean(on_values):.2f} ms\n"
                f"Paired reduction: {np.mean(reductions):+.2f} ms\n"
                f"95% CI [{lo:+.2f}, {hi:+.2f}]"
            )
            axis.text(
                0.97, 0.96, annotation, transform=axis.transAxes,
                ha="right", va="top", fontsize=11,
                bbox={"boxstyle": "square,pad=0.28", "facecolor": "white",
                      "edgecolor": "0.35", "alpha": 1.0},
            )
        fig.tight_layout(pad=0.35, w_pad=2.0)
        _save_png_pdf(fig, output, preserve_canvas=True)
        plt.close(fig)


def _paper_table_rows(pairs: list[dict]) -> list[dict]:
    rows = []
    for metric, label, unit, higher_is_better in PAPER_TABLE_METRICS:
        paired = _paired_improvements(pairs, metric, higher_is_better)
        if not paired:
            continue
        values = np.asarray(paired, dtype=float)
        on_values = values[:, 1]
        off_values = values[:, 2]
        improvements = values[:, 3]
        lo, hi = _metric_bootstrap_ci(improvements, metric)
        if metric == "success":
            off_successes = int(np.count_nonzero(off_values > 50.0))
            on_successes = int(np.count_nonzero(on_values > 50.0))
            off_lo, off_hi = _wilson(off_successes, len(off_values))
            on_lo, on_hi = _wilson(on_successes, len(on_values))
            nominal = f"{np.mean(off_values):.1f} [{off_lo:.1f}, {off_hi:.1f}]"
            proposed = f"{np.mean(on_values):.1f} [{on_lo:.1f}, {on_hi:.1f}]"
            improvement = f"{np.mean(improvements):+.1f} [{lo:+.1f}, {hi:+.1f}] pp"
        else:
            nominal = f"{np.mean(off_values):.4f} ± {np.std(off_values, ddof=1):.4f}"
            proposed = f"{np.mean(on_values):.4f} ± {np.std(on_values, ddof=1):.4f}"
            improvement = f"{np.mean(improvements):+.4f} [{lo:+.4f}, {hi:+.4f}]"
        rows.append(
            {
                "metric": label,
                "unit": unit,
                "nominal": nominal,
                "proposed": proposed,
                "paired_improvement_ci95": improvement,
                "n_pairs": len(paired),
            }
        )
    return rows


def _write_paper_table(pairs: list[dict], output_dir: Path) -> Path:
    rows = _paper_table_rows(pairs)
    csv_path = output_dir / "table1_monte_carlo_results.csv"
    fields = ["metric", "unit", "nominal", "proposed", "paired_improvement_ci95", "n_pairs"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    def latex_escape(value):
        return str(value).replace("%", r"\%").replace("±", r"$\pm$")

    latex_lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Quantitative Monte Carlo results. Baseline denotes Robust OFF and Proposed denotes Robust ON. Values are mean $\pm$ standard deviation, except success rates, which show Wilson 95\% confidence intervals. Positive paired improvement favors Proposed.}",
        r"\label{tab:monte_carlo_results}",
        r"\begin{tabular}{llccc}",
        r"\toprule",
        r"Metric & Unit & Baseline & Proposed & Paired improvement [95\% CI] \\",
        r"\midrule",
    ]
    for row in rows:
        latex_lines.append(
            " & ".join(
                latex_escape(row[field])
                for field in ("metric", "unit", "nominal", "proposed", "paired_improvement_ci95")
            ) + r" \\"
        )
    latex_lines.extend((r"\bottomrule", r"\end{tabular}", r"\end{table}"))
    (output_dir / "table1_monte_carlo_results.tex").write_text(
        "\n".join(latex_lines) + "\n", encoding="utf-8"
    )

    image_path = output_dir / "table1_monte_carlo_results.png"
    fig, axis = plt.subplots(figsize=(15.5, 4.05))
    axis.axis("off")
    column_labels = [
        "Metric", "Unit", "Baseline", "Proposed", "Paired improvement [95% CI]",
    ]
    cell_text = [
        [row["metric"], row["unit"], row["nominal"], row["proposed"],
         row["paired_improvement_ci95"]]
        for row in rows
    ]
    table = axis.table(
        cellText=cell_text, colLabels=column_labels, cellLoc="center", loc="center",
        colWidths=[0.24, 0.08, 0.20, 0.20, 0.28],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.75)
    for (row_index, column_index), cell in table.get_celld().items():
        cell.set_edgecolor("#C6CAD2")
        if row_index == 0:
            cell.set_facecolor("#E9EDF3")
            cell.set_text_props(fontweight="bold")
        elif row_index % 2 == 0:
            cell.set_facecolor("#F6F7F9")
        else:
            cell.set_facecolor("white")
        if column_index == 0:
            cell.set_text_props(ha="left")
    axis.set_title(
        "Table 1. Quantitative Monte Carlo results (50 paired samples)",
        loc="left", fontsize=14, fontweight="bold", pad=18,
    )
    axis.text(
        0.0, 0.02,
        "Baseline = Robust OFF; Proposed = Robust ON. Mean ± SD; success uses Wilson 95% CI. "
        "Positive paired improvement favors Proposed.",
        transform=axis.transAxes, fontsize=9, color="0.35", va="bottom",
    )
    fig.tight_layout()
    _save_png_pdf(fig, image_path)
    plt.close(fig)
    return image_path


def _plot_paired(pairs: list[dict], output: Path, edges):
    fig, axes = plt.subplots(2, 3, figsize=(17, 9.5), squeeze=False)
    for axis, (metric, title, unit, higher_is_better) in zip(axes.flat, PAIRED_METRICS):
        rows = _paired_improvements(pairs, metric, higher_is_better)
        if not rows:
            axis.set_title(title)
            axis.text(0.5, 0.5, "Metric unavailable", transform=axis.transAxes,
                      ha="center", va="center", color="0.4")
            axis.set_axis_off()
            continue
        values = np.asarray(rows, dtype=float)
        offsets = values[:, 0]
        improvements = values[:, 3]
        lo, hi = _metric_bootstrap_ci(improvements, metric)
        mean = float(np.mean(improvements))
        axis.scatter(offsets, improvements, color=ON_COLOR, alpha=0.55, s=28,
                     edgecolors="white", linewidth=0.35)
        centers, means, lower, upper, _counts = _bin_series(offsets, improvements, edges)
        if centers.size:
            axis.plot(centers, means, color=ON_COLOR, linewidth=2.0, marker="o",
                      markersize=4.5)
            axis.fill_between(centers, lower, upper, color=ON_COLOR, alpha=0.13)
        axis.axhline(0.0, color="0.25", linestyle="--", linewidth=1.0)
        axis.set_title(f"{title}\nmean {mean:+.3g} [{lo:+.3g}, {hi:+.3g}]")
        axis.set_xlabel("Terrain perception error [cm]")
        axis.set_ylabel(f"ON improvement [{unit}]")
        _style_axis(axis)
    fig.suptitle(
        "Paired Robust ON improvement — positive values always favor ON\n"
        "points: matched samples; lines: offset-bin means; title brackets: bootstrap 95% CI",
        fontsize=15, fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(output, dpi=160)
    plt.close(fig)


def _plot_timing(pairs: list[dict], output: Path, edges) -> bool:
    available = any(
        _metric_value(pair[mode], metric) is not None
        for pair in pairs for mode in ("ON", "OFF") for metric, _title, _unit in TIMING_METRICS
    )
    if not available:
        return False
    fig, axes = plt.subplots(2, 2, figsize=(15, 9.5), squeeze=False)
    for axis, (metric, title, unit) in zip(axes.flat, TIMING_METRICS):
        _plot_response(axis, pairs, metric, title, unit, edges)
        if metric.endswith("deadline_rate_pct"):
            axis.set_ylim(-5.0, 105.0)
    fig.suptitle(
        "Monte Carlo MPC/WBC timing response\n"
        "points: trials; lines: offset-bin means; bands: 95% mean intervals; red ×: failed trial",
        fontsize=15, fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return True


def _format_csv(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.8g}"
    return value


def _write_tables(pairs: list[dict], output_dir: Path):
    raw_fields = ["sample_id", "sample_seed", "offset_m", "first_mode", "pair_complete"]
    for metric, _title, _unit, _higher in PAIRED_METRICS:
        raw_fields.extend((f"{metric}_on", f"{metric}_off", f"{metric}_on_improvement"))
    raw_rows = []
    for pair in pairs:
        row = {
            "sample_id": pair["sample_id"],
            "sample_seed": pair["sample_seed"],
            "offset_m": pair["offset_m"],
            "first_mode": pair["first_mode"],
            "pair_complete": pair["ON"] is not None and pair["OFF"] is not None,
        }
        for metric, _title, _unit, higher_is_better in PAIRED_METRICS:
            on_value = _metric_value(pair["ON"], metric)
            off_value = _metric_value(pair["OFF"], metric)
            row[f"{metric}_on"] = on_value
            row[f"{metric}_off"] = off_value
            row[f"{metric}_on_improvement"] = (
                None if on_value is None or off_value is None
                else (on_value - off_value if higher_is_better else off_value - on_value)
            )
        raw_rows.append(row)
    with (output_dir / "monte_carlo_pairs.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=raw_fields)
        writer.writeheader()
        for row in raw_rows:
            writer.writerow({field: _format_csv(row.get(field)) for field in raw_fields})

    summary_fields = [
        "metric", "label", "unit", "n_pairs", "on_mean", "off_mean",
        "mean_on_improvement", "median_on_improvement", "bootstrap_ci95_low",
        "bootstrap_ci95_high",
    ]
    summary_rows = []
    for metric, label, unit, higher_is_better in PAIRED_METRICS:
        rows = _paired_improvements(pairs, metric, higher_is_better)
        if rows:
            values = np.asarray(rows, dtype=float)
            improvements = values[:, 3]
            lo, hi = _metric_bootstrap_ci(improvements, metric)
            summary_rows.append(
                {
                    "metric": metric,
                    "label": label,
                    "unit": unit,
                    "n_pairs": len(rows),
                    "on_mean": float(np.mean(values[:, 1])),
                    "off_mean": float(np.mean(values[:, 2])),
                    "mean_on_improvement": float(np.mean(improvements)),
                    "median_on_improvement": float(np.median(improvements)),
                    "bootstrap_ci95_low": lo,
                    "bootstrap_ci95_high": hi,
                }
            )
    with (output_dir / "monte_carlo_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        for row in summary_rows:
            writer.writerow({field: _format_csv(row.get(field)) for field in summary_fields})


def generate_monte_carlo_outputs(
    records: list[dict], manifest_path: Path, output_dir: Path
) -> list[Path]:
    """Create Monte Carlo dashboards/tables and return gallery image paths."""
    samples = _load_manifest(manifest_path)
    pairs = _pair_trials(records, samples)
    if not pairs:
        raise RuntimeError(f"No Monte Carlo samples found in {manifest_path}")

    offsets_cm = np.asarray([100.0 * pair["offset_m"] for pair in pairs], dtype=float)
    minimum, maximum = float(np.min(offsets_cm)), float(np.max(offsets_cm))
    if math.isclose(minimum, maximum):
        minimum -= 0.5
        maximum += 0.5
    bin_count = min(8, max(4, int(round(math.sqrt(len(pairs))))))
    edges = np.linspace(minimum, maximum, bin_count + 1)

    output_dir.mkdir(parents=True, exist_ok=True)
    paper_figure2 = output_dir / "fig2_computational_performance.png"
    overview = output_dir / "monte_carlo_overview.png"
    paired = output_dir / "monte_carlo_paired_improvement.png"
    timing = output_dir / "monte_carlo_timing.png"
    paper_figure1_outputs = _plot_paper_figure1(pairs, output_dir, edges)
    for stale_figure1 in (
        output_dir / "fig1_monte_carlo_robustness.png",
        output_dir / "fig1_monte_carlo_robustness.pdf",
    ):
        stale_figure1.unlink(missing_ok=True)
    paper_table = _write_paper_table(pairs, output_dir)
    _plot_paper_figure2(pairs, paper_figure2)
    _plot_overview(pairs, overview, edges)
    _plot_paired(pairs, paired, edges)
    has_timing = _plot_timing(pairs, timing, edges)
    if not has_timing and timing.exists():
        timing.unlink()
    _write_tables(pairs, output_dir)
    return [
        *paper_figure1_outputs,
        paper_table,
        paper_figure2,
        overview,
        paired,
        *([timing] if has_timing else []),
    ]
