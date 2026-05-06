#!/usr/bin/env python3
"""
Plot a 2x2 grid summary from a noise_sweep.py summary.csv.

Layout (rows x cols):
  (0,0) Success rate vs terrain_z_offset
  (0,1) Mean time_to_failure vs terrain_z_offset (failures only, error bars)
  (1,0) Mean distance_xy vs terrain_z_offset
  (1,1) Mean roll_rms_deg vs terrain_z_offset

Lines: arm=raw (red), arm=refined (blue).
Error bars: 1 standard deviation across repetitions, when n >= 2.

Usage:
    python3 plot_sweep_summary.py <out-dir>/summary.csv [--save out.png]
"""

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

# Allow non-GUI environments (sweep runs on a headless server, etc.)
if "DISPLAY" not in __import__("os").environ:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ARM_COLORS = {"raw": "tab:red", "refined": "tab:blue"}


def _parse_float(value):
    if value is None or value == "":
        return None
    try:
        x = float(value)
    except ValueError:
        return None
    if math.isnan(x) or math.isinf(x):
        return None
    return x


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    s = str(value).strip().lower()
    return s in ("1", "true", "yes", "y", "t")


def load_rows(csv_path: Path):
    rows = []
    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            noise = _parse_float(r.get("noise"))
            if noise is None:
                continue
            rows.append(
                {
                    "noise": noise,
                    "arm": r.get("arm", "").strip() or "raw",
                    "rep": int(_parse_float(r.get("rep")) or 0),
                    "success": _parse_bool(r.get("success")),
                    "fall_reason": (r.get("fall_reason") or "").strip() or None,
                    "time_to_failure": _parse_float(r.get("time_to_failure")),
                    "distance_xy": _parse_float(r.get("distance_xy")),
                    "body_forward_path_length": _parse_float(r.get("body_forward_path_length")),
                    "roll_rms_deg": _parse_float(r.get("roll_rms_deg")),
                }
            )
    return rows


def aggregate(rows, *, value_key, success_filter=None):
    """Return {arm: (xs, means, stds)} sorted by noise.

    success_filter:
        None       -> include all rows
        True       -> include rows where success is True
        False      -> include rows where success is False
    """
    buckets = defaultdict(lambda: defaultdict(list))  # arm -> noise -> [values]
    for row in rows:
        if success_filter is not None and row["success"] != success_filter:
            continue
        v = row.get(value_key)
        if v is None:
            continue
        buckets[row["arm"]][row["noise"]].append(v)
    out = {}
    for arm, by_noise in buckets.items():
        xs = sorted(by_noise.keys())
        means = []
        stds = []
        for x in xs:
            vals = by_noise[x]
            n = len(vals)
            mean = sum(vals) / n
            if n >= 2:
                var = sum((v - mean) ** 2 for v in vals) / (n - 1)
                std = math.sqrt(var)
            else:
                std = 0.0
            means.append(mean)
            stds.append(std)
        out[arm] = (xs, means, stds)
    return out


def aggregate_success_rate(rows):
    """Return {arm: (xs, rates)} where rate is fraction of success=True."""
    buckets = defaultdict(lambda: defaultdict(list))
    for row in rows:
        buckets[row["arm"]][row["noise"]].append(1 if row["success"] else 0)
    out = {}
    for arm, by_noise in buckets.items():
        xs = sorted(by_noise.keys())
        rates = [sum(by_noise[x]) / len(by_noise[x]) for x in xs]
        out[arm] = (xs, rates)
    return out


def plot_panel(ax, series_by_arm, *, title, ylabel, errorbars=False):
    for arm, payload in series_by_arm.items():
        color = ARM_COLORS.get(arm, "black")
        if errorbars:
            xs, means, stds = payload
            ax.errorbar(xs, means, yerr=stds, marker="o", color=color, label=arm, capsize=3)
        else:
            xs, ys = payload[0], payload[1]
            ax.plot(xs, ys, marker="o", color=color, label=arm)
    ax.set_xlabel("terrain_z_offset (m)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.axvline(0.0, color="gray", linestyle=":", linewidth=0.8)
    ax.legend(fontsize=8)


def main():
    parser = argparse.ArgumentParser(description="Render a 2x2 summary plot from a noise_sweep summary.csv")
    parser.add_argument("summary_csv", help="Path to summary.csv produced by noise_sweep.py")
    parser.add_argument("--save", default=None, help="If set, write PNG here. Otherwise plt.show().")
    parser.add_argument("--title", default=None, help="Optional figure suptitle.")
    args = parser.parse_args()

    csv_path = Path(args.summary_csv).expanduser().resolve()
    if not csv_path.exists():
        print(f"[plot_sweep_summary] not found: {csv_path}", file=sys.stderr)
        sys.exit(2)

    rows = load_rows(csv_path)
    if not rows:
        print(f"[plot_sweep_summary] no data rows in {csv_path}", file=sys.stderr)
        sys.exit(1)

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    # (0,0) success rate
    plot_panel(
        axes[0, 0],
        {arm: (xs, rates, [0.0] * len(rates)) for arm, (xs, rates) in aggregate_success_rate(rows).items()},
        title="Success rate",
        ylabel="success rate",
        errorbars=False,
    )
    axes[0, 0].set_ylim(-0.05, 1.05)

    # (0,1) mean time_to_failure (failures only)
    plot_panel(
        axes[0, 1],
        aggregate(rows, value_key="time_to_failure", success_filter=False),
        title="Mean time_to_failure (failures only)",
        ylabel="time_to_failure (s)",
        errorbars=True,
    )

    # (1,0) mean distance_xy
    plot_panel(
        axes[1, 0],
        aggregate(rows, value_key="distance_xy"),
        title="Mean distance_xy",
        ylabel="distance_xy (m)",
        errorbars=True,
    )

    # (1,1) mean roll_rms_deg
    plot_panel(
        axes[1, 1],
        aggregate(rows, value_key="roll_rms_deg"),
        title="Mean roll_rms_deg",
        ylabel="roll RMS (deg)",
        errorbars=True,
    )

    if args.title:
        fig.suptitle(args.title)
    fig.tight_layout()

    if args.save:
        save_path = Path(args.save).expanduser().resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=140)
        print(f"[plot_sweep_summary] wrote {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
