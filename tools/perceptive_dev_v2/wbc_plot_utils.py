#!/usr/bin/env python3
"""Shared trial discovery and time-window helpers for WBC plots."""

import json
import re
from pathlib import Path

import yaml


def _load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _as_bool(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def discover_trials(results_dir: Path) -> list[dict]:
    """Discover completed/usable trials and infer their A/B condition."""
    trials = []
    for trial_dir in sorted(path for path in results_dir.iterdir() if path.is_dir()):
        tick_path = trial_dir / "tick.csv"
        if trial_dir.name == "all_visualizations" or not tick_path.is_file():
            continue
        if tick_path.stat().st_size == 0:
            continue

        config = _load_json(trial_dir / "run_config.json")
        result = _load_json(trial_dir / "result.json")
        effective = config.get("effective_task_parameters", {}).get("robustPhase", {})
        if not effective:
            effective = result.get("experiment", {}).get("effective_task_parameters", {}).get(
                "robustPhase", {}
            )

        enabled = effective.get("enabled")
        if enabled is None:
            robust = "ON" if re.search(r"(?:^|_)ON(?:_|$)|robON", trial_dir.name) else "OFF"
        else:
            robust = "ON" if _as_bool(enabled) else "OFF"

        offset = config.get("terrain_z_offset")
        if offset is None:
            offset = result.get("experiment", {}).get("terrain_z_offset")
        if offset is None:
            match = re.search(r"off([MP])(\d+)", trial_dir.name)
            if match:
                magnitude = int(match.group(2)) / (10 ** len(match.group(2)))
                offset = magnitude if match.group(1) == "P" else -magnitude
            elif re.search(r"(?:^|_)off0(?:_|$)", trial_dir.name):
                offset = 0.0
        if offset is None:
            continue

        run_match = re.search(r"_run(\d+)(?:_|$)", trial_dir.name)
        run = int(run_match.group(1)) if run_match else 1
        trials.append(
            {
                "trial_dir": trial_dir,
                "tick_path": tick_path,
                "offset_m": float(offset),
                "robust": robust,
                "run": run,
                "config": config,
                "result": result,
            }
        )
    return trials


def group_trials(trials: list[dict]) -> dict[tuple[float, str], list[dict]]:
    groups = {}
    for trial in trials:
        key = (trial["offset_m"], trial["robust"])
        groups.setdefault(key, []).append(trial)
    for grouped in groups.values():
        grouped.sort(key=lambda item: (item["run"], item["trial_dir"].name))
    return groups


def command_window_in_tick_time(trial: dict, t_rel_end: float) -> tuple[float, float]:
    """Return the commanded-motion window relative to the first OCS2 tick.

    OCS2 tick logging starts when the enter_ocs2 command is issued. The experiment
    metrics store the command-active window in scenario time, so the scenario
    YAML lets us convert that window into tick-relative time. If old metadata is
    incomplete, use the full available tick range.
    """
    result = trial.get("result", {})
    active_start = result.get("command_active_window_start_sec")
    active_end = result.get("command_active_window_end_sec")
    scenario_candidates = []
    configured_scenario = trial.get("config", {}).get("scenario")
    if configured_scenario:
        scenario_candidates.append(Path(configured_scenario))
    scenario_candidates.append(trial["trial_dir"] / "scenario.yaml")
    scenario_name = result.get("scenario")
    if scenario_name:
        for candidate in sorted((Path(__file__).parent / "scenarios").glob("*.yaml")):
            try:
                candidate_data = yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}
            except (OSError, yaml.YAMLError):
                continue
            if candidate_data.get("name") == scenario_name:
                scenario_candidates.append(candidate)
                break

    entry_start = None
    for scenario_path in scenario_candidates:
        if not scenario_path.is_file():
            continue
        try:
            scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8")) or {}
            elapsed = 0.0
            for step in scenario.get("steps", []):
                if step.get("name") == "enter_ocs2":
                    entry_start = elapsed
                    break
                elapsed += float(step.get("duration", 0.0))
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            entry_start = None
        if entry_start is not None:
            break

    if active_start is None or active_end is None or entry_start is None:
        return 0.0, max(0.0, t_rel_end)

    start = max(0.0, float(active_start) - entry_start)
    end = min(float(t_rel_end), float(active_end) - entry_start)
    if end <= start:
        return 0.0, max(0.0, t_rel_end)
    return start, end


def offset_label(offset: float) -> str:
    return f"Δz={offset:+.2f}"


def offset_file_token(offset: float) -> str:
    return f"{offset:+.2f}".replace(".", "")
