#!/usr/bin/env python3
"""Plot MPC computation time and solve frequencies from a trial log."""

import argparse
import json
import re
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


MPC_TIMING_RE = re.compile(
    r"\[(?P<timestamp>\d+(?:\.\d+)?)\].*?\[MPC timing\] "
    r"target_hz=(?P<target_hz>[-+\d.eE]+) "
    r"solve_hz=(?P<solve_hz>[-+\d.eE]+) "
    r"capacity_hz=(?P<capacity_hz>[-+\d.eE]+) "
    r"solve_ms\(last/avg/max\)="
    r"(?P<solve_ms_last>[-+\d.eE]+)/"
    r"(?P<solve_ms_avg>[-+\d.eE]+)/"
    r"(?P<solve_ms_max>[-+\d.eE]+) "
    r"load=(?P<load_pct>[-+\d.eE]+)%"
    r"(?: sqp_iter\(last/avg/max/limit\)="
    r"(?P<sqp_iter_last>[-+\d.eE]+)/"
    r"(?P<sqp_iter_avg>[-+\d.eE]+)/"
    r"(?P<sqp_iter_max>[-+\d.eE]+)/"
    r"(?P<sqp_iter_limit>[-+\d.eE]+))?"
    r"(?: deadline_hit=(?P<deadline_hits>\d+)/(?P<deadline_solves>\d+)"
    r" deadline_rate=(?P<deadline_rate_pct>[-+\d.eE]+)%)?"
)

TIMING_FIELDS = (
    "timestamp",
    "target_hz",
    "solve_hz",
    "capacity_hz",
    "solve_ms_last",
    "solve_ms_avg",
    "solve_ms_max",
    "load_pct",
    "sqp_iter_last",
    "sqp_iter_avg",
    "sqp_iter_max",
    "sqp_iter_limit",
    "deadline_hits",
    "deadline_solves",
    "deadline_rate_pct",
)


def parse_mpc_timing(log_path: Path) -> dict[str, np.ndarray]:
    """Return MPC timing samples parsed from ``controller.log``."""
    samples = {field: [] for field in TIMING_FIELDS}
    with log_path.open(encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            match = MPC_TIMING_RE.search(line)
            if match is None:
                continue
            for field in TIMING_FIELDS:
                value = match.group(field)
                samples[field].append(float(value) if value is not None else np.nan)
    return {field: np.asarray(values, dtype=float) for field, values in samples.items()}


def plot_mpc_timing(trial_dir: Path, output: Path | None = None) -> Path:
    log_path = trial_dir / "controller.log"
    if not log_path.is_file():
        raise FileNotFoundError(f"missing controller.log in {trial_dir}")

    timing = parse_mpc_timing(log_path)
    if timing["timestamp"].size == 0:
        raise RuntimeError(f"no [MPC timing] samples found in {log_path}")

    elapsed = timing["timestamp"] - timing["timestamp"][0]
    deadline_ms = np.divide(
        1000.0,
        timing["target_hz"],
        out=np.full_like(timing["target_hz"], np.nan),
        where=timing["target_hz"] > 0.0,
    )

    has_sqp_iterations = bool(np.isfinite(timing["sqp_iter_avg"]).any())
    num_axes = 3 if has_sqp_iterations else 2
    fig, axes = plt.subplots(num_axes, 1, figsize=(12, 10 if has_sqp_iterations else 8), sharex=True)
    axes[0].plot(elapsed, timing["solve_ms_last"], marker="o", ms=3,
                 linewidth=1.1, label="last solve")
    axes[0].plot(elapsed, timing["solve_ms_avg"], linewidth=1.8,
                 label="rolling average")
    axes[0].plot(elapsed, timing["solve_ms_max"], linewidth=1.0,
                 alpha=0.75, label="rolling maximum")
    axes[0].plot(elapsed, deadline_ms, linestyle="--", color="black",
                 linewidth=1.1, label="target period")
    axes[0].set_ylabel("Computation time [ms]")
    axes[0].set_title("MPC solve computation time")
    axes[0].grid(alpha=0.3)
    axes[0].legend(ncol=4, fontsize=9)

    axes[1].plot(elapsed, timing["target_hz"], linestyle="--", color="black",
                 linewidth=1.2, label="target Hz")
    axes[1].plot(elapsed, timing["solve_hz"], marker="o", ms=3,
                 linewidth=1.3, label="achieved solve Hz")
    axes[1].plot(elapsed, timing["capacity_hz"], linewidth=1.5,
                 label="solver capacity Hz")
    axes[1].set_ylabel("Frequency [Hz]")
    axes[1].set_title("MPC target, achieved rate, and solver capacity")
    axes[1].grid(alpha=0.3)
    axes[1].legend(ncol=3, fontsize=9)

    if has_sqp_iterations:
        valid = np.isfinite(timing["sqp_iter_avg"])
        axes[2].plot(elapsed[valid], timing["sqp_iter_last"][valid], marker="o", ms=3,
                     linewidth=1.1, label="last SQP iterations")
        axes[2].plot(elapsed[valid], timing["sqp_iter_avg"][valid], linewidth=1.8,
                     label="rolling average")
        axes[2].plot(elapsed[valid], timing["sqp_iter_max"][valid], linewidth=1.0,
                     alpha=0.75, label="rolling maximum")
        axes[2].plot(elapsed[valid], timing["sqp_iter_limit"][valid], linestyle="--",
                     color="black", linewidth=1.1, label="configured limit")
        axes[2].set_ylabel("SQP iterations")
        axes[2].set_title("SQP iterations per MPC solve")
        axes[2].grid(alpha=0.3)
        axes[2].legend(ncol=4, fontsize=9)

    axes[-1].set_xlabel("Time since first MPC timing sample [s]")

    mean_ms = float(np.mean(timing["solve_ms_avg"]))
    p95_ms = float(np.percentile(timing["solve_ms_last"], 95))
    mean_hz = float(np.mean(timing["solve_hz"]))
    mean_capacity_hz = float(np.mean(timing["capacity_hz"]))
    mean_load = float(np.mean(timing["load_pct"]))
    stats = {
        "samples": int(timing["timestamp"].size),
        "solve_ms_mean": mean_ms,
        "solve_ms_last_p95": p95_ms,
        "solve_hz_mean": mean_hz,
        "capacity_hz_mean": mean_capacity_hz,
        "load_pct_mean": mean_load,
    }
    exact_deadline = np.isfinite(timing["deadline_hits"]) & np.isfinite(
        timing["deadline_solves"]
    )
    if np.any(exact_deadline) and np.sum(timing["deadline_solves"][exact_deadline]) > 0:
        deadline_hits = int(np.sum(timing["deadline_hits"][exact_deadline]))
        deadline_solves = int(np.sum(timing["deadline_solves"][exact_deadline]))
        stats.update({
            "deadline_hits": deadline_hits,
            "deadline_solves": deadline_solves,
            "deadline_rate_pct": 100.0 * deadline_hits / deadline_solves,
            "deadline_method": "exact_per_solve",
        })
    else:
        valid_deadline = np.isfinite(deadline_ms) & np.isfinite(timing["solve_ms_last"])
        if np.any(valid_deadline):
            stats.update({
                "deadline_rate_pct": float(
                    100.0 * np.mean(
                        timing["solve_ms_last"][valid_deadline]
                        <= deadline_ms[valid_deadline]
                    )
                ),
                "deadline_method": "legacy_last_sample_estimate",
            })
    iteration_title = ""
    if has_sqp_iterations:
        valid = np.isfinite(timing["sqp_iter_avg"])
        mean_iterations = float(np.mean(timing["sqp_iter_avg"][valid]))
        max_iterations = int(np.max(timing["sqp_iter_max"][valid]))
        iteration_limit = int(np.max(timing["sqp_iter_limit"][valid]))
        stats.update({
            "sqp_iteration_samples": int(np.count_nonzero(valid)),
            "sqp_iterations_mean": mean_iterations,
            "sqp_iterations_max": max_iterations,
            "sqp_iteration_limit": iteration_limit,
        })
        iteration_title = (
            f", SQP iter={mean_iterations:.2f} mean/{max_iterations} max/"
            f"{iteration_limit} limit"
        )
    fig.suptitle(
        f"{trial_dir.name}\n"
        f"mean solve={mean_ms:.2f} ms, p95 last={p95_ms:.2f} ms, "
        f"solve={mean_hz:.2f} Hz, capacity={mean_capacity_hz:.2f} Hz, "
        f"load={mean_load:.1f}%{iteration_title}",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    output = output or trial_dir / "mpc_timing.png"
    fig.savefig(output, dpi=140)
    plt.close(fig)
    (trial_dir / "mpc_timing.json").write_text(
        json.dumps(stats, indent=2), encoding="utf-8"
    )
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        output = plot_mpc_timing(args.trial_dir.resolve(), args.output)
    except (FileNotFoundError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
