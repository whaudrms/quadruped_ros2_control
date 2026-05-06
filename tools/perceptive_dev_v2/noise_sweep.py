#!/usr/bin/env python3
"""
Noise sweep harness for the perceptive_dev_v2 trial runner.

For each (terrain_z_offset, arm, repetition) combo, invokes run_trial.py with
the appropriate flags, parses the resulting result.json, and appends a row to
summary.csv.

Arms:
  raw     — OCS2 only (no refiner)
  refined — OCS2 + robust_refine daemon (--enable-refiner)

Each trial runs under a hard wall-clock timeout. On timeout the trial is
killed, the lingering-process nuke is invoked, and the trial is logged as
fall_reason=timeout.

Example:
    python3 noise_sweep.py \
        --noise-values 0.00 0.02 -0.02 0.05 -0.05 \
        --arms raw refined \
        --repetitions 2

Output layout:
    <out-dir>/
        summary.csv          (one row per trial)
        sweep.log            (stdout/stderr from this driver)
        trials/<run-name>/   (forwarded run_trial.py result tree)
"""

import argparse
import csv
import datetime as _dt
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parent
RESULTS_DIR = ROOT / "results"
RUN_TRIAL = ROOT / "run_trial.py"

# Mirror the lingering-process patterns used by run_trial.py's defensive
# cleanup. Kept in sync deliberately rather than imported, because run_trial.py
# pulls in heavy deps (yaml) and we want noise_sweep.py to stay light.
LINGERING_PROCESS_PATTERNS = (
    "unitree_mujoco",
    "ros2_control_node",
    "ocs2_quadruped_controller",
    "ros2 launch ocs2",
    "planar_terrain",
    "refiner_daemon.py",
)


def kill_lingering_processes(quiet: bool = False) -> None:
    """SIGKILL any leftover trial processes. Tolerates pkill missing-match exit."""
    for pattern in LINGERING_PROCESS_PATTERNS:
        try:
            result = subprocess.run(
                ["pkill", "-9", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if not quiet and result.returncode == 0:
                print(f"[noise_sweep] killed leftover '{pattern}'")
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            if not quiet:
                print(f"[noise_sweep] pkill {pattern!r} failed: {exc}")


def find_latest_result_json(run_tag: str, since: float) -> Path | None:
    """Find the result.json that run_trial.py produced for this tag.

    run_trial.py creates ``results/<timestamp>_<scene>_<mode>_<tag>/result.json``.
    The simplest robust way is to scan ``results/`` for directories whose name
    contains the tag and whose mtime is >= ``since``.
    """
    if not RESULTS_DIR.exists():
        return None
    candidates = []
    for child in RESULTS_DIR.iterdir():
        if not child.is_dir():
            continue
        if run_tag not in child.name:
            continue
        result_json = child / "result.json"
        if result_json.exists() and result_json.stat().st_mtime >= since - 1.0:
            candidates.append(result_json)
    if not candidates:
        return None
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def run_one_trial(
    *,
    noise: float,
    arm: str,
    rep: int,
    timeout_sec: float,
    extra_run_trial_args: Iterable[str],
    sweep_log,
) -> dict:
    """Invoke run_trial.py once and return a summary dict."""
    tag = f"sweep_z{noise:+.3f}_{arm}_r{rep}"
    cmd = [
        sys.executable,
        str(RUN_TRIAL),
        "--terrain-z-offset",
        f"{noise}",
        "--tag",
        tag,
    ]
    if arm == "refined":
        cmd.append("--enable-refiner")
    cmd.extend(extra_run_trial_args)

    print(f"[noise_sweep] starting trial tag={tag}", file=sweep_log, flush=True)
    print(f"[noise_sweep] cmd={' '.join(cmd)}", file=sweep_log, flush=True)

    t0 = time.monotonic()
    started_wall = time.time()
    timed_out = False
    returncode = None
    try:
        proc = subprocess.run(
            cmd,
            stdout=sweep_log,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
            start_new_session=True,
        )
        returncode = proc.returncode
    except subprocess.TimeoutExpired as te:
        timed_out = True
        print(
            f"[noise_sweep] tag={tag} TIMED OUT after {timeout_sec:.1f}s; killing",
            file=sweep_log,
            flush=True,
        )
        # subprocess.run already attempted termination, but the trial spawns a
        # whole tree; let the lingering-process nuke clean up.
        try:
            os.killpg(os.getpgid(te.pid or 0), signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass
    elapsed = time.monotonic() - t0

    # Always nuke between trials, even on success — defends against any
    # zombies that escaped run_trial.py's own cleanup.
    kill_lingering_processes(quiet=True)

    # Locate result.json.
    result_json_path = find_latest_result_json(tag, since=started_wall)
    if timed_out or result_json_path is None:
        # No result.json: synthesize a timeout / failure row.
        return {
            "noise": noise,
            "arm": arm,
            "rep": rep,
            "tag": tag,
            "success": False,
            "fall_reason": "timeout" if timed_out else "no_result_json",
            "time_to_failure": None,
            "distance_xy": None,
            "body_forward_path_length": None,
            "roll_rms_deg": None,
            "run_dir": "",
            "elapsed_wall": elapsed,
            "returncode": returncode,
        }
    try:
        with open(result_json_path, "r", encoding="utf-8") as f:
            result = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[noise_sweep] failed to read {result_json_path}: {exc}", file=sweep_log, flush=True)
        return {
            "noise": noise,
            "arm": arm,
            "rep": rep,
            "tag": tag,
            "success": False,
            "fall_reason": "result_json_unreadable",
            "time_to_failure": None,
            "distance_xy": None,
            "body_forward_path_length": None,
            "roll_rms_deg": None,
            "run_dir": str(result_json_path.parent),
            "elapsed_wall": elapsed,
            "returncode": returncode,
        }
    return {
        "noise": noise,
        "arm": arm,
        "rep": rep,
        "tag": tag,
        "success": bool(result.get("success", False)),
        "fall_reason": result.get("fall_reason"),
        "time_to_failure": result.get("time_to_failure"),
        "distance_xy": result.get("distance_xy"),
        "body_forward_path_length": result.get("body_forward_path_length"),
        "roll_rms_deg": result.get("roll_rms_deg"),
        "run_dir": str(result_json_path.parent),
        "elapsed_wall": elapsed,
        "returncode": returncode,
    }


def append_summary_row(summary_csv: Path, row: dict) -> None:
    fieldnames = [
        "timestamp",
        "noise",
        "arm",
        "rep",
        "tag",
        "success",
        "fall_reason",
        "time_to_failure",
        "distance_xy",
        "body_forward_path_length",
        "roll_rms_deg",
        "run_dir",
        "elapsed_wall",
        "returncode",
    ]
    summary_csv.parent.mkdir(parents=True, exist_ok=True)
    file_exists = summary_csv.exists()
    with open(summary_csv, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def format_progress(idx: int, total: int, row: dict) -> str:
    noise = row["noise"]
    arm = row["arm"]
    rep = row["rep"]
    if row["success"]:
        outcome = "OK"
    else:
        ttf = row.get("time_to_failure")
        outcome = f"fall@{ttf:.1f}s" if isinstance(ttf, (int, float)) else f"fail({row.get('fall_reason')})"
    dx = row.get("distance_xy")
    dx_str = f"{dx:.2f}m" if isinstance(dx, (int, float)) else "n/a"
    roll = row.get("roll_rms_deg")
    roll_str = f"{roll:.1f}deg" if isinstance(roll, (int, float)) else "n/a"
    return (
        f"[{idx}/{total}] noise={noise:+.3f} arm={arm} rep={rep} "
        f"-> {outcome} dx={dx_str} roll={roll_str}"
    )


def main():
    parser = argparse.ArgumentParser(description="Run a terrain-noise x refiner-arm sweep over run_trial.py")
    parser.add_argument(
        "--noise-values",
        type=float,
        nargs="+",
        default=[0.00, 0.02, -0.02, 0.05, -0.05],
        help="Values for --terrain-z-offset to sweep (meters).",
    )
    parser.add_argument(
        "--arms",
        nargs="+",
        choices=["raw", "refined"],
        default=["raw", "refined"],
        help="raw=OCS2 only, refined=OCS2 + robust_refine daemon.",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=1,
        help="Number of repetitions per (noise, arm) cell.",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory. Default: results_sweep_<timestamp>/ next to this script.",
    )
    parser.add_argument(
        "--trial-timeout-sec",
        type=float,
        default=90.0,
        help="Hard wall-clock timeout for each run_trial.py invocation.",
    )
    parser.add_argument(
        "--run-trial-arg",
        action="append",
        default=[],
        help="Extra argument to forward to run_trial.py. Pass repeatedly, e.g. "
             "--run-trial-arg=--terrain --run-trial-arg=basic_step.xml",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the planned trials but don't execute them.",
    )
    args = parser.parse_args()

    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.out_dir is None:
        out_dir = ROOT / f"results_sweep_{timestamp}"
    else:
        out_dir = Path(args.out_dir).expanduser().resolve()

    # Build the trial plan: outer loop = noise, middle = arm, inner = rep.
    # This ordering keeps consecutive trials at the same noise level so a flaky
    # noise level fails fast.
    plan = [
        (noise, arm, rep)
        for noise in args.noise_values
        for arm in args.arms
        for rep in range(args.repetitions)
    ]
    total = len(plan)

    print(f"[noise_sweep] out_dir={out_dir}")
    print(f"[noise_sweep] {total} trials planned")
    if args.dry_run:
        for i, (noise, arm, rep) in enumerate(plan, 1):
            print(f"  [{i}/{total}] noise={noise:+.3f} arm={arm} rep={rep}")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = out_dir / "summary.csv"
    sweep_log_path = out_dir / "sweep.log"

    # Defensive cleanup before the sweep starts.
    kill_lingering_processes(quiet=True)

    with open(sweep_log_path, "w", encoding="utf-8") as sweep_log:
        for idx, (noise, arm, rep) in enumerate(plan, 1):
            row = run_one_trial(
                noise=noise,
                arm=arm,
                rep=rep,
                timeout_sec=args.trial_timeout_sec,
                extra_run_trial_args=args.run_trial_arg,
                sweep_log=sweep_log,
            )
            row["timestamp"] = _dt.datetime.now().isoformat(timespec="seconds")
            append_summary_row(summary_csv, row)
            print(format_progress(idx, total, row), flush=True)

    print(f"[noise_sweep] done. summary={summary_csv}")


if __name__ == "__main__":
    main()
