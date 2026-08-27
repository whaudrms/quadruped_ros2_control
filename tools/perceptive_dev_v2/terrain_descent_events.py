#!/usr/bin/env python3
"""Detect step-down events from saved OCS2 ticks and measured foot FK."""

import csv
import json
from pathlib import Path

import numpy as np
import pinocchio as pin


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_URDF = PROJECT_ROOT / "descriptions/unitree/go2_description/urdf/robot.urdf"
LEG_NAMES = ("FL", "FR", "RL", "RR")
FRAME_NAMES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
DETECTOR_VERSION = 1

# basic_step_short_v2 physical geometry. The URDF foot frame sits about 0.06 m
# above the contact point, so a lower-surface touchdown appears near z=0.16 m.
EDGE_X_M = 0.60
EDGE_X_TOLERANCE_M = 0.05
UPPER_SURFACE_Z_M = 0.20
LOWER_SURFACE_Z_M = 0.10
FOOT_FRAME_OFFSET_M = 0.06
LOWER_TOUCHDOWN_Z_MAX_M = 0.21


def euler_zyx_to_quat_xyzw(theta_zyx: np.ndarray) -> np.ndarray:
    yaw, pitch, roll = map(float, theta_zyx)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    return np.array(
        [
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr,
            cy * cp * cr + sy * sp * sr,
        ],
        dtype=float,
    )


def load_detection_columns(tick_path: Path):
    with tick_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        n_expected = len(header)
        t_col = header.index("t")
        meas_cols = [header.index(f"meas_rbd{i}") for i in range(36)]
        mode_col = header.index("planned_mode")
        times, measured, modes = [], [], []
        for row in reader:
            if len(row) != n_expected:
                continue
            try:
                values = [float(value) for value in row]
            except ValueError:
                continue
            times.append(values[t_col])
            measured.append([values[column] for column in meas_cols])
            modes.append(int(values[mode_col]))
    if not times:
        raise RuntimeError(f"No valid samples in {tick_path}")
    return np.asarray(times), np.asarray(measured), np.asarray(modes, dtype=int)


def compute_measured_foot_xyz(measured: np.ndarray, urdf_path: Path = DEFAULT_URDF):
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()
    frame_ids = [model.getFrameId(name) for name in FRAME_NAMES]
    if any(frame_id >= model.nframes for frame_id in frame_ids):
        raise RuntimeError("One or more GO2 foot frames are missing from the URDF")

    foot_xyz = np.zeros((measured.shape[0], 4, 3))
    q = np.zeros(model.nq)
    for index, state in enumerate(measured):
        q[0:3] = state[3:6]
        q[3:7] = euler_zyx_to_quat_xyzw(state[0:3])
        q[7:19] = state[6:18]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        for leg, frame_id in enumerate(frame_ids):
            foot_xyz[index, leg] = data.oMf[frame_id].translation
    return foot_xyz


def _event(index: int, times: np.ndarray, foot_xyz: np.ndarray, modes: np.ndarray, leg: int):
    return {
        "raw_time_sec": float(times[index]),
        "tick_relative_sec": float(times[index] - times[0]),
        "foot_x_m": float(foot_xyz[index, leg, 0]),
        "foot_z_m": float(foot_xyz[index, leg, 2]),
        "planned_mode": int(modes[index]),
    }


def detect_events(tick_path: Path, urdf_path: Path = DEFAULT_URDF) -> dict:
    times, measured, modes = load_detection_columns(tick_path)
    foot_xyz = compute_measured_foot_xyz(measured, urdf_path)
    feet = {}

    for leg, leg_name in enumerate(LEG_NAMES):
        contact = (modes >> (3 - leg)) & 1
        touchdown_indices = np.flatnonzero((contact[:-1] == 0) & (contact[1:] == 1)) + 1
        edge_indices = np.flatnonzero(foot_xyz[:, leg, 0] >= EDGE_X_M)
        lower_touchdowns = [
            int(index)
            for index in touchdown_indices
            if foot_xyz[index, leg, 0] >= EDGE_X_M - EDGE_X_TOLERANCE_M
            and foot_xyz[index, leg, 2] <= LOWER_TOUCHDOWN_Z_MAX_M
        ]
        feet[leg_name] = {
            "edge_crossing": _event(int(edge_indices[0]), times, foot_xyz, modes, leg)
            if edge_indices.size
            else None,
            "lower_touchdown": _event(lower_touchdowns[0], times, foot_xyz, modes, leg)
            if lower_touchdowns
            else None,
        }

    front_edges = [feet[name]["edge_crossing"] for name in ("FL", "FR")]
    front_touchdowns = [feet[name]["lower_touchdown"] for name in ("FL", "FR")]
    hind_touchdowns = [feet[name]["lower_touchdown"] for name in ("RL", "RR")]
    front_edges = [event for event in front_edges if event]
    front_touchdowns = [event for event in front_touchdowns if event]
    hind_touchdowns = [event for event in hind_touchdowns if event]

    entry = min(front_edges, key=lambda event: event["raw_time_sec"]) if front_edges else None
    start = (
        min(front_touchdowns, key=lambda event: event["raw_time_sec"])
        if front_touchdowns
        else None
    )
    complete = (
        max(hind_touchdowns, key=lambda event: event["raw_time_sec"])
        if hind_touchdowns
        else None
    )
    duration = None
    if start and complete and complete["raw_time_sec"] >= start["raw_time_sec"]:
        duration = complete["raw_time_sec"] - start["raw_time_sec"]

    stat = tick_path.stat()
    return {
        "detector_version": DETECTOR_VERSION,
        "tick_csv": str(tick_path.resolve()),
        "tick_size": stat.st_size,
        "tick_mtime_ns": stat.st_mtime_ns,
        "geometry": {
            "edge_x_m": EDGE_X_M,
            "edge_x_tolerance_m": EDGE_X_TOLERANCE_M,
            "upper_surface_z_m": UPPER_SURFACE_Z_M,
            "lower_surface_z_m": LOWER_SURFACE_Z_M,
            "foot_frame_offset_m": FOOT_FRAME_OFFSET_M,
            "lower_touchdown_z_max_m": LOWER_TOUCHDOWN_Z_MAX_M,
        },
        "method": "planned swing-to-stance and measured-foot FK position",
        "feet": feet,
        "descent": {
            "front_edge_entry": entry,
            "first_lower_touchdown": start,
            "last_hind_lower_touchdown": complete,
            "duration_sec": float(duration) if duration is not None else None,
        },
    }


def load_or_detect_events(trial_dir: Path, force: bool = False) -> dict:
    tick_path = trial_dir / "tick.csv"
    output_path = trial_dir / "terrain_descent_events.json"
    if not tick_path.is_file():
        raise FileNotFoundError(tick_path)
    stat = tick_path.stat()
    if output_path.is_file() and not force:
        try:
            cached = json.loads(output_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            cached = {}
        if (
            cached.get("detector_version") == DETECTOR_VERSION
            and cached.get("tick_size") == stat.st_size
            and cached.get("tick_mtime_ns") == stat.st_mtime_ns
        ):
            return cached
    events = detect_events(tick_path)
    output_path.write_text(json.dumps(events, indent=2), encoding="utf-8")
    return events


def descent_times(events: dict, time_key: str = "tick_relative_sec"):
    descent = events.get("descent", {})
    start = descent.get("first_lower_touchdown")
    complete = descent.get("last_hind_lower_touchdown")
    return (
        start.get(time_key) if start else None,
        complete.get(time_key) if complete else None,
    )
