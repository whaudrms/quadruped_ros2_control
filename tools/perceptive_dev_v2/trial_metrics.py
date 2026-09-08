#!/usr/bin/env python3
"""Extract touchdown, WBC timing, and robust-band metrics from one saved trial.

New tick logs contain measured contact mode and per-cycle WBC timing.  Older logs
remain usable: planned contact transitions and tick spacing are used as explicitly
labelled fallbacks.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from plot_robust_phase import extract_unique_windows, parse_robust_phase_log
from terrain_descent_events import LEG_NAMES, compute_measured_foot_xyz


METRIC_VERSION = 2
TOUCHDOWN_VELOCITY_WINDOW_S = 0.020
ROBUST_EVENT_MATCH_TOLERANCE_S = 0.15


def _load_tick(tick_path: Path):
    with tick_path.open(encoding="utf-8") as stream:
        header = stream.readline().strip().split(",")
    rows = []
    expected = len(header)
    with tick_path.open(encoding="utf-8") as stream:
        next(stream)
        for line in stream:
            values = line.rstrip().split(",")
            if len(values) != expected:
                continue
            try:
                rows.append([float(value) for value in values])
            except ValueError:
                continue
    if not rows:
        raise RuntimeError(f"no valid rows in {tick_path}")
    return header, np.asarray(rows, dtype=float)


def _mode_contacts(modes: np.ndarray, leg: int) -> np.ndarray:
    # OCS2 stanceLeg2ModeNumber maps FL,FR,RL,RR to bits 3,2,1,0.
    return ((modes.astype(np.int64) >> (3 - leg)) & 1).astype(bool)


def _touchdown_events(times, modes, foot_xyz, mode_source: str) -> list[dict]:
    events = []
    for leg, leg_name in enumerate(LEG_NAMES):
        contact = _mode_contacts(modes, leg)
        indices = np.flatnonzero((~contact[:-1]) & contact[1:]) + 1
        for index in indices:
            event_time = float(times[index])
            fit_indices = np.flatnonzero(
                (times >= event_time - TOUCHDOWN_VELOCITY_WINDOW_S)
                & (times < event_time)
            )
            velocity = np.full(3, np.nan)
            if fit_indices.size >= 3:
                fit_time = times[fit_indices] - event_time
                for axis in range(3):
                    velocity[axis] = np.polyfit(
                        fit_time, foot_xyz[fit_indices, leg, axis], 1
                    )[0]
            events.append(
                {
                    "leg": leg_name,
                    "leg_index": leg,
                    "time_s": event_time,
                    "foot_xyz_m": foot_xyz[index, leg].tolist(),
                    "velocity_xyz_mps": velocity.tolist(),
                    "touchdown_normal_speed_mps": (
                        float(max(0.0, -velocity[2]))
                        if np.isfinite(velocity[2]) else None
                    ),
                    "touchdown_speed_mps": (
                        float(np.linalg.norm(velocity))
                        if np.all(np.isfinite(velocity)) else None
                    ),
                    "contact_source": mode_source,
                }
            )
    return sorted(events, key=lambda event: event["time_s"])


def _runtime_metrics(header, tick, foot_xyz) -> tuple[dict, list[dict]]:
    times = tick[:, header.index("t")]
    if "measured_mode" in header:
        mode_column = "measured_mode"
        mode_source = "measured_contact"
    else:
        mode_column = "planned_mode"
        mode_source = "planned_contact_fallback"
    modes = tick[:, header.index(mode_column)].astype(int)
    touchdown_events = _touchdown_events(times, modes, foot_xyz, mode_source)
    touchdown_normal = [
        event["touchdown_normal_speed_mps"] for event in touchdown_events
        if event["touchdown_normal_speed_mps"] is not None
    ]

    dt = np.diff(times)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    achieved_hz = (
        float((times.size - 1) / (times[-1] - times[0]))
        if times.size > 1 and times[-1] > times[0] else None
    )
    if "control_period_s" in header:
        control_period = tick[:, header.index("control_period_s")]
        valid_period = control_period[np.isfinite(control_period) & (control_period > 0.0)]
        target_period_s = float(np.median(valid_period)) if valid_period.size else 0.001
    else:
        control_period = None
        # Legacy files do not record the configured controller period.  Infer
        # the nearest integer rate from the observed tick spacing.
        inferred_hz = round(1.0 / float(np.median(dt))) if dt.size else 500
        target_period_s = 1.0 / max(1, inferred_hz)

    runtime = {
        "metric_version": METRIC_VERSION,
        "contact_source": mode_source,
        "touchdown_events": touchdown_events,
        "touchdown_event_count": len(touchdown_events),
        "touchdown_normal_speed_mean_mps": (
            float(np.mean(touchdown_normal)) if touchdown_normal else None
        ),
        "touchdown_normal_speed_p95_mps": (
            float(np.percentile(touchdown_normal, 95)) if touchdown_normal else None
        ),
        "wbc_target_hz": 1.0 / target_period_s,
        "wbc_achieved_hz": achieved_hz,
        "wbc_tick_period_mean_ms": float(1000.0 * np.mean(dt)) if dt.size else None,
        "wbc_tick_period_p95_ms": float(1000.0 * np.percentile(dt, 95)) if dt.size else None,
    }

    if "wbc_solve_ms" in header:
        solve_ms = tick[:, header.index("wbc_solve_ms")]
        deadlines_ms = (
            1000.0 * control_period
            if control_period is not None
            else np.full_like(solve_ms, 1000.0 * target_period_s)
        )
        valid = (
            np.isfinite(solve_ms) & np.isfinite(deadlines_ms) & (deadlines_ms > 0.0)
        )
        if np.any(valid):
            runtime.update(
                {
                    "wbc_solve_ms_mean": float(np.mean(solve_ms[valid])),
                    "wbc_solve_ms_p95": float(np.percentile(solve_ms[valid], 95)),
                    "wbc_deadline_rate_pct": float(
                        100.0 * np.mean(solve_ms[valid] <= deadlines_ms[valid])
                    ),
                    "wbc_deadline_samples": int(np.count_nonzero(valid)),
                    "wbc_deadline_method": "exact_compute_time_per_cycle",
                }
            )
    elif dt.size:
        runtime.update(
            {
                "wbc_deadline_rate_pct": float(
                    100.0 * np.mean(dt <= 1.05 * target_period_s)
                ),
                "wbc_deadline_samples": int(dt.size),
                "wbc_deadline_method": "legacy_tick_period_estimate_5pct_tolerance",
            }
        )
    return runtime, touchdown_events


def _nearest_value(times: np.ndarray, values: np.ndarray, query: float) -> float:
    valid = np.isfinite(times) & np.isfinite(values)
    if not np.any(valid):
        return float("nan")
    valid_indices = np.flatnonzero(valid)
    index = valid_indices[int(np.argmin(np.abs(times[valid] - query)))]
    return float(values[index])


def _robust_metrics(
    controller_log: Path,
    times: np.ndarray,
    foot_xyz: np.ndarray,
    touchdown_events: list[dict],
) -> dict:
    base = {
        "metric_version": METRIC_VERSION,
        "available": False,
        "method": "measured foot-frame FK, event-matched robust windows",
        "event_match_tolerance_s": ROBUST_EVENT_MATCH_TOLERANCE_S,
    }
    if not controller_log.is_file():
        return {**base, "unavailable_reason": "missing_controller_log"}
    windows = extract_unique_windows(parse_robust_phase_log(controller_log))
    if not any(windows.values()):
        return {**base, "unavailable_reason": "no_active_robust_windows"}

    samples = []
    used_windows = set()
    for event in touchdown_events:
        leg = int(event["leg_index"])
        candidates = []
        for window_index, window in enumerate(windows[leg]):
            ta, tb, _pz, _d, _offset, _clamped = window
            distance = abs(float(tb) - float(event["time_s"]))
            if distance <= ROBUST_EVENT_MATCH_TOLERANCE_S:
                candidates.append((distance, window_index, window))
        if not candidates:
            continue
        _distance, window_index, window = min(candidates, key=lambda item: item[0])
        window_key = (leg, window_index)
        if window_key in used_windows:
            continue
        used_windows.add(window_key)
        ta, tb, pz, d, offset, clamped = window
        d = float(d)
        if d <= 0.0:
            continue
        in_window = (times >= ta) & (times <= tb)
        if np.count_nonzero(in_window) < 2:
            continue
        guard = foot_xyz[:, leg, 2] - float(pz) - float(offset)
        window_guard = guard[in_window]
        window_guard = window_guard[np.isfinite(window_guard)]
        if window_guard.size < 2:
            continue
        guard_min = float(np.min(window_guard))
        guard_max = float(np.max(window_guard))
        covered = max(0.0, min(guard_max, d) - max(guard_min, -d))
        coverage_ratio = float(np.clip(covered / (2.0 * d), 0.0, 1.0))
        end_guard = _nearest_value(times, guard, float(tb))
        end_violation = max(0.0, end_guard + d) / d
        violations = [end_violation]
        start_guard = None
        start_violation = None
        if not clamped:
            start_guard = _nearest_value(times, guard, float(ta))
            start_violation = max(0.0, d - start_guard) / d
            violations.append(start_violation)
        contact_guard = _nearest_value(times, guard, float(event["time_s"]))
        if not np.isfinite(end_guard) or not np.isfinite(contact_guard):
            continue
        if start_guard is not None and not np.isfinite(start_guard):
            continue
        samples.append(
            {
                "leg": event["leg"],
                "contact_time_s": event["time_s"],
                "contact_source": event["contact_source"],
                "ta_s": float(ta),
                "tb_s": float(tb),
                "d_m": d,
                "clamped": bool(clamped),
                "guard_min_m": guard_min,
                "guard_max_m": guard_max,
                "start_guard_m": start_guard,
                "end_guard_m": end_guard,
                "contact_guard_m": contact_guard,
                "band_traversed": bool(
                    (not clamped) and guard_max >= d and guard_min <= -d
                ),
                "uncertainty_coverage_ratio": coverage_ratio,
                "normalized_boundary_violation": float(np.mean(violations)),
                "contact_in_band": bool(-d <= contact_guard <= d),
            }
        )

    if not samples:
        return {**base, "unavailable_reason": "no_touchdown_matched_robust_window"}
    full_samples = [sample for sample in samples if not sample["clamped"]]
    contact_sources = sorted({sample["contact_source"] for sample in samples})
    return {
        **base,
        "available": True,
        "contact_sources": contact_sources,
        "matched_touchdown_count": len(samples),
        "full_window_count": len(full_samples),
        "robust_band_traversal_rate_pct": (
            float(100.0 * np.mean([sample["band_traversed"] for sample in full_samples]))
            if full_samples else None
        ),
        "uncertainty_coverage_ratio": float(
            np.mean([sample["uncertainty_coverage_ratio"] for sample in samples])
        ),
        "normalized_boundary_violation": float(
            np.mean([sample["normalized_boundary_violation"] for sample in samples])
        ),
        "contact_in_band_rate_pct": float(
            100.0 * np.mean([sample["contact_in_band"] for sample in samples])
        ),
        "samples": samples,
    }


def _fingerprint(path: Path) -> dict:
    stat = path.stat()
    return {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}


def _cache_valid(path: Path, fingerprints: dict) -> bool:
    if not path.is_file():
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return data.get("metric_version") == METRIC_VERSION and data.get("inputs") == fingerprints


def analyze_trial(trial_dir: Path, force: bool = False) -> tuple[dict, dict]:
    trial_dir = trial_dir.resolve()
    tick_path = trial_dir / "tick.csv"
    controller_log = trial_dir / "controller.log"
    if not tick_path.is_file():
        raise FileNotFoundError(tick_path)
    fingerprints = {"tick.csv": _fingerprint(tick_path)}
    if controller_log.is_file():
        fingerprints["controller.log"] = _fingerprint(controller_log)
    runtime_path = trial_dir / "runtime_metrics.json"
    robust_path = trial_dir / "robust_metrics.json"
    if (
        not force
        and _cache_valid(runtime_path, fingerprints)
        and _cache_valid(robust_path, fingerprints)
    ):
        return (
            json.loads(runtime_path.read_text(encoding="utf-8")),
            json.loads(robust_path.read_text(encoding="utf-8")),
        )

    header, tick = _load_tick(tick_path)
    measured_columns = [header.index(f"meas_rbd{i}") for i in range(36)]
    foot_xyz = compute_measured_foot_xyz(tick[:, measured_columns])
    runtime, touchdown_events = _runtime_metrics(header, tick, foot_xyz)
    robust = _robust_metrics(
        controller_log,
        tick[:, header.index("t")],
        foot_xyz,
        touchdown_events,
    )
    runtime["inputs"] = fingerprints
    robust["inputs"] = fingerprints
    runtime_path.write_text(json.dumps(runtime, indent=2), encoding="utf-8")
    robust_path.write_text(json.dumps(robust, indent=2), encoding="utf-8")
    return runtime, robust


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    try:
        runtime, robust = analyze_trial(args.trial_dir, args.force)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(
        f"saved {args.trial_dir / 'runtime_metrics.json'} "
        f"and {args.trial_dir / 'robust_metrics.json'}; "
        f"touchdowns={runtime['touchdown_event_count']}, "
        f"robust_available={robust['available']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
