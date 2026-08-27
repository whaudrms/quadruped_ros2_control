#!/usr/bin/env python3
"""Create publication plots and CSV data from a recorded perceptive trial.

The figures use the logged OCS2 state, measured robot state, planned contact
mode, MuJoCo terrain geometry, and the exact CubicSpline/SplineCpg equations
used by SwingTrajectoryPlanner.  No horizontal foot path is invented.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pinocchio as pin

from render_perceptive_preprocessing import build_terrain, load_scene


LEG_NAMES = ("FL", "FR", "RL", "RR")
FRAME_NAMES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
CONTACT_MASKS = np.array((8, 4, 2, 1), dtype=int)
LEG_COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")
PLAN_COLOR = "#0072B2"
MEASURED_COLOR = "#D55E00"
NOMINAL_COLOR = "#777777"
PERCEPTIVE_COLOR = "#009E73"
GRID_COLOR = "#D9D9D9"
FONT_SCALE = 1.5
DETAIL_FONT_SIZE = 8.2 * FONT_SCALE

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_URDF = PROJECT_ROOT / "quadruped_ros2_control/descriptions/unitree/go2_description/urdf/robot.urdf"


def configure_publication_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": [
                "Times New Roman",
                "Times",
                "Nimbus Roman",
                "Liberation Serif",
                "STIXGeneral",
            ],
            "font.size": 8.5 * FONT_SCALE,
            "axes.labelsize": 9 * FONT_SCALE,
            "axes.titlesize": 9 * FONT_SCALE,
            "legend.fontsize": 7.5 * FONT_SCALE,
            "xtick.labelsize": 8 * FONT_SCALE,
            "ytick.labelsize": 8 * FONT_SCALE,
            "mathtext.fontset": "stix",
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.5,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.04,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.unicode_minus": False,
        }
    )


def load_tick(tick_path: Path) -> tuple[list[str], np.ndarray]:
    with tick_path.open(encoding="utf-8") as stream:
        header = stream.readline().strip().split(",")
        expected = len(header)
        rows = []
        for line in stream:
            fields = line.strip().split(",")
            if len(fields) != expected:
                continue
            try:
                rows.append([float(value) for value in fields])
            except ValueError:
                continue
    if not rows:
        raise RuntimeError(f"No valid rows in {tick_path}")
    return header, np.asarray(rows, dtype=float)


def column_indices(header: list[str], prefix: str, count: int) -> list[int]:
    return [header.index(f"{prefix}{index}") for index in range(count)]


def euler_zyx_to_quaternion(theta_zyx: np.ndarray) -> np.ndarray:
    yaw, pitch, roll = theta_zyx
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    return np.array(
        [
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr,
            cy * cp * cr + sy * sp * sr,
        ]
    )


def build_configuration_trajectories(opt_state: np.ndarray, measured_rbd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    count = opt_state.shape[0]
    q_opt = np.zeros((count, 19))
    q_measured = np.zeros((count, 19))
    for index in range(count):
        q_opt[index, :3] = opt_state[index, 6:9]
        q_opt[index, 3:7] = euler_zyx_to_quaternion(opt_state[index, 9:12])
        q_opt[index, 7:19] = opt_state[index, 12:24]
        q_measured[index, :3] = measured_rbd[index, 3:6]
        q_measured[index, 3:7] = euler_zyx_to_quaternion(measured_rbd[index, 0:3])
        q_measured[index, 7:19] = measured_rbd[index, 6:18]
    return q_opt, q_measured


def compute_foot_positions(urdf_path: Path, configurations: np.ndarray) -> np.ndarray:
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()
    frame_ids = [model.getFrameId(name) for name in FRAME_NAMES]
    if any(frame_id >= model.nframes for frame_id in frame_ids):
        raise RuntimeError("A Go2 foot frame is missing from the URDF")
    output = np.zeros((configurations.shape[0], 4, 3))
    for sample, configuration in enumerate(configurations):
        pin.forwardKinematics(model, data, configuration)
        pin.updateFramePlacements(model, data)
        for leg, frame_id in enumerate(frame_ids):
            output[sample, leg] = data.oMf[frame_id].translation
    return output


def parse_info_value(text: str, key: str, default: float) -> float:
    match = re.search(rf"^\s*{re.escape(key)}\s+([-+0-9.eE]+)", text, flags=re.MULTILINE)
    return float(match.group(1)) if match else default


def load_swing_config(trial_dir: Path) -> dict[str, float]:
    task_path = trial_dir / "task.info.effective"
    text = task_path.read_text(encoding="utf-8") if task_path.is_file() else ""
    return {
        "position_error_gain": parse_info_value(text, "positionErrorGain", 0.0),
        "lift_off_velocity": parse_info_value(text, "liftOffVelocity", 0.05),
        "touch_down_velocity": parse_info_value(text, "touchDownVelocity", -0.10),
        "swing_height": parse_info_value(text, "swingHeight", 0.08),
        "swing_time_scale": parse_info_value(text, "swingTimeScale", 0.15),
    }


def load_smoothing_radius(trial_dir: Path) -> float:
    log_path = trial_dir / "controller.log"
    if log_path.is_file():
        match = re.search(r"smoothing_radius=([-+0-9.eE]+)", log_path.read_text(encoding="utf-8", errors="ignore"))
        if match:
            return float(match.group(1))
    return 0.06


def contact_flags(modes: np.ndarray) -> np.ndarray:
    return (modes[:, None] & CONTACT_MASKS[None, :]) != 0


def nearest_time_index(times: np.ndarray, event_time: float) -> int:
    return int(np.argmin(np.abs(times - event_time)))


def touchdown_events(
    trial_dir: Path,
    times: np.ndarray,
    contacts: np.ndarray,
    planned_xyz: np.ndarray,
    measured_xyz: np.ndarray,
) -> list[dict]:
    event_path = trial_dir / "terrain_descent_events.json"
    events_json = json.loads(event_path.read_text(encoding="utf-8")) if event_path.is_file() else {}
    output = []
    for leg, name in enumerate(LEG_NAMES):
        event = events_json.get("feet", {}).get(name, {}).get("lower_touchdown")
        if event:
            index = nearest_time_index(times, float(event["raw_time_sec"]))
        else:
            candidates = np.flatnonzero((~contacts[:-1, leg]) & contacts[1:, leg]) + 1
            candidates = [
                int(index)
                for index in candidates
                if measured_xyz[index, leg, 0] >= 0.55 and measured_xyz[index, leg, 2] <= 0.23
            ]
            if not candidates:
                continue
            index = candidates[0]
        output.append(
            {
                "leg": leg,
                "leg_name": name,
                "index": index,
                "time_sec": float(times[index]),
                "planned_xyz": planned_xyz[index, leg].copy(),
                "measured_xyz": measured_xyz[index, leg].copy(),
            }
        )
    if not output:
        raise RuntimeError("No lower-surface touchdown was detected")
    return output


def swing_bounds(contacts: np.ndarray, leg: int, touchdown_index: int) -> tuple[int, int]:
    start = touchdown_index - 1
    while start > 0 and not contacts[start - 1, leg]:
        start -= 1
    if contacts[start, leg]:
        start += 1
    return start, touchdown_index


def sample_grid(data, x: np.ndarray | float, y: np.ndarray | float, layer: str = "elevation") -> np.ndarray:
    x_values = np.asarray(x)
    y_values = np.asarray(y)
    ix = np.clip(np.searchsorted(data.x, x_values), 0, len(data.x) - 1)
    iy = np.clip(np.searchsorted(data.y, y_values), 0, len(data.y) - 1)
    values = data.elevation if layer == "elevation" else data.smooth
    return values[iy, ix]


def cubic_spline(
    time: np.ndarray,
    start_time: float,
    start_position: float,
    start_velocity: float,
    final_time: float,
    final_position: float,
    final_velocity: float,
) -> tuple[np.ndarray, np.ndarray]:
    duration = final_time - start_time
    normalized = (time - start_time) / duration
    delta_position = final_position - start_position
    delta_velocity = final_velocity - start_velocity
    dc1 = start_velocity
    dc2 = -(3.0 * start_velocity + delta_velocity)
    dc3 = 2.0 * start_velocity + delta_velocity
    c0 = start_position
    c1 = dc1 * duration
    c2 = dc2 * duration + 3.0 * delta_position
    c3 = dc3 * duration - 2.0 * delta_position
    position = c3 * normalized**3 + c2 * normalized**2 + c1 * normalized + c0
    velocity = (3.0 * c3 * normalized**2 + 2.0 * c2 * normalized + c1) / duration
    return position, velocity


def spline_cpg(
    time: np.ndarray,
    start_time: float,
    final_time: float,
    lift_off_height: float,
    touch_down_height: float,
    mid_height: float,
    lift_off_velocity: float,
    touch_down_velocity: float,
) -> tuple[np.ndarray, np.ndarray]:
    mid_time = 0.5 * (start_time + final_time)
    position = np.empty_like(time)
    velocity = np.empty_like(time)
    left = time < mid_time
    position[left], velocity[left] = cubic_spline(
        time[left], start_time, lift_off_height, lift_off_velocity, mid_time, mid_height, 0.0
    )
    position[~left], velocity[~left] = cubic_spline(
        time[~left], mid_time, mid_height, 0.0, final_time, touch_down_height, touch_down_velocity
    )
    return position, velocity


def save_figure(fig: plt.Figure, output_base: Path) -> None:
    fig.savefig(output_base.with_suffix(".pdf"))
    fig.savefig(output_base.with_suffix(".png"), dpi=300)
    plt.close(fig)


def plot_footholds(perceived_terrain, events: list[dict], output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    extent = [perceived_terrain.x[0], perceived_terrain.x[-1], perceived_terrain.y[0], perceived_terrain.y[-1]]
    image = ax.imshow(
        perceived_terrain.smooth,
        origin="lower",
        extent=extent,
        cmap="Blues",
        vmin=float(perceived_terrain.smooth.min()),
        vmax=float(perceived_terrain.smooth.max()),
        aspect="equal",
        interpolation="nearest",
    )
    label_offsets = {"FL": (-18, 9), "FR": (7, 7), "RL": (7, -13), "RR": (-18, -13)}
    for event in events:
        leg = event["leg"]
        planned = event["planned_xyz"]
        measured = event["measured_xyz"]
        color = LEG_COLORS[leg]
        ax.scatter(planned[0], planned[1], s=46, marker="o", facecolor="none", edgecolor=color, linewidth=1.5)
        ax.scatter(measured[0], measured[1], s=42, marker="x", color=color, linewidth=1.5)
        ax.annotate(
            event["leg_name"],
            xy=(planned[0], planned[1]),
            xytext=label_offsets[event["leg_name"]],
            textcoords="offset points",
            color=color,
            fontsize=7.5 * FONT_SCALE,
        )
    ax.set_xlim(-0.1, 1.35)
    ax.set_ylim(-0.65, 0.65)
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_title("Lower-surface touchdown footholds from the recorded OCS2 trial", loc="left")
    ax.grid(color=GRID_COLOR, linewidth=0.5, alpha=0.65)
    colorbar = fig.colorbar(image, ax=ax, fraction=0.035, pad=0.025)
    colorbar.set_label("perceived terrain height [m]")
    shape_legend = [
        Line2D([], [], marker="o", markerfacecolor="none", markeredgecolor="black", linestyle="none", label="OCS2 planned"),
        Line2D([], [], marker="x", color="black", linestyle="none", label="measured"),
    ]
    ax.legend(handles=shape_legend, frameon=False, loc="lower right", ncol=2)
    fig.tight_layout()
    save_figure(fig, output_dir / "paper_foothold_map")


def plot_swing(
    phase: np.ndarray,
    planned: np.ndarray,
    measured: np.ndarray,
    nominal_z: np.ndarray,
    nominal_vz: np.ndarray,
    perceptive_z: np.ndarray,
    perceptive_vz: np.ndarray,
    physical_height: np.ndarray,
    perceived_height: np.ndarray,
    leg_name: str,
    output_dir: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 7.0), gridspec_kw={"height_ratios": [1.0, 0.85, 1.1]})
    position_ax, velocity_ax, path_ax = axes

    position_ax.plot(phase, nominal_z, color=NOMINAL_COLOR, linestyle="--", label="same-height nominal SplineCpg")
    position_ax.plot(phase, perceptive_z, color=PERCEPTIVE_COLOR, label="perceptive SplineCpg (internal)")
    position_ax.plot(phase, planned[:, 2], color=PLAN_COLOR, label="OCS2 planned foot FK")
    position_ax.plot(phase, measured[:, 2], color=MEASURED_COLOR, alpha=0.85, label="measured foot FK")
    position_ax.set_ylabel("$z$ [m]")
    position_ax.set_title(f"(a) {leg_name} swing height", loc="left")
    position_ax.legend(frameon=False, ncol=2, loc="upper right")

    velocity_ax.plot(phase, nominal_vz, color=NOMINAL_COLOR, linestyle="--", label="same-height nominal")
    velocity_ax.plot(phase, perceptive_vz, color=PERCEPTIVE_COLOR, label="perceptive")
    velocity_ax.axhline(0.0, color="black", linewidth=0.6)
    velocity_ax.set_ylabel("$\dot{z}_{ref}$ [m/s]")
    velocity_ax.set_title("(b) Exact SwingTrajectoryPlanner vertical-velocity reference", loc="left")
    velocity_ax.legend(frameon=False, ncol=2, loc="upper right")

    order = np.argsort(planned[:, 0])
    path_ax.fill_between(
        planned[order, 0],
        0.0,
        physical_height[order],
        color="#E6E6E6",
        step="mid",
        label="physical terrain",
    )
    path_ax.plot(planned[order, 0], perceived_height[order], color="black", linestyle=":", label="perceived terrain")
    path_ax.plot(planned[:, 0], planned[:, 2], color=PLAN_COLOR, label="OCS2 planned foot FK")
    path_ax.plot(measured[:, 0], measured[:, 2], color=MEASURED_COLOR, alpha=0.85, label="measured foot FK")
    path_ax.scatter(planned[-1, 0], planned[-1, 2], marker="o", facecolor="white", edgecolor=PLAN_COLOR, zorder=5)
    path_ax.scatter(measured[-1, 0], measured[-1, 2], marker="x", color=MEASURED_COLOR, zorder=5)
    path_ax.set_xlabel("$x$ [m]")
    path_ax.set_ylabel("$z$ [m]")
    path_ax.set_title("(c) Recorded sagittal swing path and terrain", loc="left")
    path_ax.legend(frameon=False, ncol=2, loc="upper right")

    for ax in axes:
        ax.grid(color=GRID_COLOR, linewidth=0.5, alpha=0.8)
        for spine in ax.spines.values():
            spine.set_linewidth(0.8)
    velocity_ax.set_xlabel("normalized swing phase")
    position_ax.set_xlim(0.0, 1.0)
    velocity_ax.set_xlim(0.0, 1.0)
    fig.tight_layout(h_pad=1.0)
    save_figure(fig, output_dir / "paper_swing_trajectory")


def surface_profile_at_y(surface, x_values: np.ndarray, y_value: float) -> tuple[np.ndarray, np.ndarray]:
    """Return the part of one planar surface intersected by an x-z cross section."""
    projection = surface.rotation[:2, :2]
    if abs(np.linalg.det(projection)) < 1e-9:
        return np.zeros_like(x_values, dtype=bool), np.full_like(x_values, np.nan)
    delta = np.column_stack(
        [x_values - surface.top_center[0], np.full_like(x_values, y_value - surface.top_center[1])]
    )
    local = delta @ np.linalg.inv(projection).T
    mask = (np.abs(local[:, 0]) <= surface.half_x + 1e-6) & (
        np.abs(local[:, 1]) <= surface.half_y + 1e-6
    )
    height = (
        surface.top_center[2]
        + surface.rotation[2, 0] * local[:, 0]
        + surface.rotation[2, 1] * local[:, 1]
    )
    return mask, height


def plot_perceptive_pipeline(
    perceived_terrain,
    planned: np.ndarray,
    nominal_z: np.ndarray,
    perceptive_z: np.ndarray,
    lift_off_height: float,
    touch_down_height: float,
    leg_name: str,
    output_dir: Path,
) -> None:
    """Plot three perceptive processing stages on one common x-z section."""
    lift_off_x = float(planned[0, 0])
    touch_down_x = float(planned[-1, 0])
    y_section = float(np.median(planned[:, 1]))
    swing_span = max(abs(touch_down_x - lift_off_x), 0.15)
    left_margin = max(0.12, 0.55 * swing_span)
    right_margin = max(0.18, 0.85 * swing_span)
    x_limits = (
        min(lift_off_x, touch_down_x) - left_margin,
        max(lift_off_x, touch_down_x) + right_margin,
    )
    x_dense = np.linspace(x_limits[0], x_limits[1], 900)
    y_dense = np.full_like(x_dense, y_section)
    raw_height = sample_grid(perceived_terrain, x_dense, y_dense, layer="elevation")
    z_bottom = min(-0.015, float(raw_height.min()) - 0.035)
    z_top = max(float(perceptive_z.max()), float(nominal_z.max()), float(planned[:, 2].max())) + 0.055

    fig, axes = plt.subplots(3, 1, figsize=(7.4, 6.4), sharex=True)

    def draw_terrain(ax: plt.Axes) -> None:
        ax.fill_between(x_dense, z_bottom, raw_height, color="#E6E6E6", step="mid", linewidth=0.0)
        ax.plot(
            x_dense,
            raw_height,
            color="black",
            linewidth=1.15,
            drawstyle="steps-mid",
            label="_nolegend_",
        )

    # 1. Planar decomposition and the two footholds bounding this swing.
    draw_terrain(axes[0])
    region_colors = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")
    if perceived_terrain.has_floor:
        axes[0].hlines(0.0, *x_limits, color="#777777", linewidth=3.0, label="floor region")
    for surface_index, surface in enumerate(perceived_terrain.surfaces):
        mask, height = surface_profile_at_y(surface, x_dense, y_section)
        if not np.any(mask):
            continue
        color = region_colors[surface_index % len(region_colors)]
        region_x = x_dense[mask]
        region_z = height[mask]
        axes[0].plot(region_x, region_z, color=color, linewidth=4.0, solid_capstyle="butt")
        inset_mask = (region_x >= region_x[0] + 0.02) & (region_x <= region_x[-1] - 0.02)
        if np.any(inset_mask):
            axes[0].plot(region_x[inset_mask], region_z[inset_mask] + 0.006, color=color, linestyle=":", linewidth=1.2)
    axes[0].scatter(
        [lift_off_x, touch_down_x],
        [lift_off_height, touch_down_height],
        s=48,
        facecolor="white",
        edgecolor=PERCEPTIVE_COLOR,
        linewidth=1.7,
        zorder=5,
    )
    axes[0].annotate(
        f"lift-off foothold\n({lift_off_x:.3f}, {lift_off_height:.3f}) m",
        (lift_off_x, lift_off_height),
        xytext=(-4, 10),
        textcoords="offset points",
        ha="right",
        fontsize=DETAIL_FONT_SIZE,
    )
    axes[0].annotate(
        f"selected touchdown\n({touch_down_x:.3f}, {touch_down_height:.3f}) m",
        (touch_down_x, touch_down_height),
        xytext=(5, 9),
        textcoords="offset points",
        ha="left",
        fontsize=DETAIL_FONT_SIZE,
    )
    axes[0].set_title(
        "1.  Terrain decomposition and foothold selection", loc="left", fontsize=10.2 * FONT_SCALE
    )

    # 2. Same-height SplineCpg before applying the selected terrain height.
    draw_terrain(axes[1])
    axes[1].plot(planned[:, 0], nominal_z, color=NOMINAL_COLOR, linestyle="--", linewidth=1.8, label="nominal trajectory")
    axes[1].scatter(
        [lift_off_x, touch_down_x],
        [nominal_z[0], nominal_z[-1]],
        s=45,
        facecolor="white",
        edgecolor=NOMINAL_COLOR,
        linewidth=1.5,
        zorder=5,
    )
    axes[1].scatter(
        touch_down_x,
        touch_down_height,
        s=42,
        marker="x",
        color=PERCEPTIVE_COLOR,
        linewidth=1.5,
        zorder=5,
        label="terrain-selected foothold",
    )
    axes[1].plot(
        [touch_down_x, touch_down_x],
        [touch_down_height, nominal_z[-1]],
        color=PERCEPTIVE_COLOR,
        linestyle=":",
        linewidth=1.0,
    )
    axes[1].set_title(
        "2.  Nominal swing trajectory", loc="left", fontsize=10.2 * FONT_SCALE
    )
    axes[1].legend(frameon=False, ncol=1, loc="upper right", fontsize=DETAIL_FONT_SIZE, labelspacing=0.35)

    # 3. Exact terrain-height-modified SplineCpg.
    draw_terrain(axes[2])
    axes[2].plot(planned[:, 0], nominal_z, color=NOMINAL_COLOR, linestyle="--", linewidth=1.0, alpha=0.75, label="nominal trajectory")
    axes[2].plot(planned[:, 0], perceptive_z, color=PERCEPTIVE_COLOR, linewidth=2.0, label="modified trajectory")
    axes[2].scatter(
        [lift_off_x, touch_down_x],
        [perceptive_z[0], perceptive_z[-1]],
        s=48,
        facecolor="white",
        edgecolor=PERCEPTIVE_COLOR,
        linewidth=1.7,
        zorder=5,
    )
    axes[2].annotate(
        f"terrain correction: {touch_down_height - nominal_z[-1]:+.3f} m",
        (touch_down_x, touch_down_height),
        xytext=(-8, -20),
        textcoords="offset points",
        ha="right",
        va="top",
        fontsize=DETAIL_FONT_SIZE,
        color=PERCEPTIVE_COLOR,
    )
    axes[2].set_title(
        "3.  Modified swing trajectory: connected to selected foothold", loc="left", fontsize=10.2 * FONT_SCALE
    )
    axes[2].legend(frameon=False, ncol=1, loc="upper right", fontsize=DETAIL_FONT_SIZE, labelspacing=0.35)
    axes[2].set_xlabel("forward position $x$ [m]")

    for index, ax in enumerate(axes):
        ax.set_xlim(*x_limits)
        ax.set_ylim(z_bottom, z_top)
        ax.set_ylabel("$z$ [m]")
        ax.grid(color=GRID_COLOR, linewidth=0.5, alpha=0.75)
        for spine in ax.spines.values():
            spine.set_linewidth(0.8)
        if index < len(axes) - 1:
            ax.tick_params(labelbottom=False)
    fig.tight_layout(rect=(0.04, 0.02, 0.995, 0.995), h_pad=0.8)
    for upper, lower in zip(axes[:-1], axes[1:]):
        upper_box = upper.get_position()
        lower_box = lower.get_position()
        fig.text(
            max(0.02, upper_box.x0 - 0.055),
            0.5 * (upper_box.y0 + lower_box.y1),
            "$\\downarrow$",
            ha="center",
            va="center",
            fontsize=11 * FONT_SCALE,
        )

    save_figure(fig, output_dir / "paper_perceptive_pipeline")


def write_foothold_csv(events: list[dict], terrain, output_path: Path) -> None:
    fields = [
        "leg",
        "touchdown_time_sec",
        "planned_x_m",
        "planned_y_m",
        "planned_z_m",
        "measured_x_m",
        "measured_y_m",
        "measured_z_m",
        "perceived_terrain_height_m",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for event in events:
            planned = event["planned_xyz"]
            measured = event["measured_xyz"]
            writer.writerow(
                {
                    "leg": event["leg_name"],
                    "touchdown_time_sec": f"{event['time_sec']:.9f}",
                    "planned_x_m": f"{planned[0]:.9f}",
                    "planned_y_m": f"{planned[1]:.9f}",
                    "planned_z_m": f"{planned[2]:.9f}",
                    "measured_x_m": f"{measured[0]:.9f}",
                    "measured_y_m": f"{measured[1]:.9f}",
                    "measured_z_m": f"{measured[2]:.9f}",
                    "perceived_terrain_height_m": f"{float(sample_grid(terrain, measured[0], measured[1])):.9f}",
                }
            )


def write_swing_csv(
    output_path: Path,
    times: np.ndarray,
    phase: np.ndarray,
    planned: np.ndarray,
    measured: np.ndarray,
    nominal_z: np.ndarray,
    nominal_vz: np.ndarray,
    perceptive_z: np.ndarray,
    perceptive_vz: np.ndarray,
    physical_height: np.ndarray,
    perceived_height: np.ndarray,
) -> None:
    fields = [
        "time_sec",
        "swing_phase",
        "nominal_spline_z_m",
        "nominal_spline_vz_mps",
        "perceptive_spline_z_m",
        "perceptive_spline_vz_mps",
        "planned_foot_x_m",
        "planned_foot_y_m",
        "planned_foot_z_m",
        "measured_foot_x_m",
        "measured_foot_y_m",
        "measured_foot_z_m",
        "physical_terrain_height_m",
        "perceived_terrain_height_m",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index in range(len(times)):
            writer.writerow(
                [
                    f"{times[index]:.9f}",
                    f"{phase[index]:.9f}",
                    f"{nominal_z[index]:.9f}",
                    f"{nominal_vz[index]:.9f}",
                    f"{perceptive_z[index]:.9f}",
                    f"{perceptive_vz[index]:.9f}",
                    *(f"{value:.9f}" for value in planned[index]),
                    *(f"{value:.9f}" for value in measured[index]),
                    f"{physical_height[index]:.9f}",
                    f"{perceived_height[index]:.9f}",
                ]
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial-dir", type=Path, required=True, help="Recorded trial containing tick.csv")
    parser.add_argument("--output-dir", type=Path, required=True, help="Destination for PDF, PNG, CSV, and metadata")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trial_dir = args.trial_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    config = json.loads((trial_dir / "run_config.json").read_text(encoding="utf-8"))
    scene_path = Path(config["terrain"])
    terrain_offset = float(config.get("terrain_z_offset", 0.0))
    offset_threshold = float(config.get("terrain_z_offset_only_below_z", math.inf))
    smoothing_radius = load_smoothing_radius(trial_dir)
    swing_config = load_swing_config(trial_dir)

    header, tick = load_tick(trial_dir / "tick.csv")
    times = tick[:, header.index("t")]
    opt_state = tick[:, column_indices(header, "opt_x", 24)]
    measured_rbd = tick[:, column_indices(header, "meas_rbd", 36)]
    modes = tick[:, header.index("planned_mode")].astype(int)
    q_opt, q_measured = build_configuration_trajectories(opt_state, measured_rbd)
    planned_xyz = compute_foot_positions(args.urdf.resolve(), q_opt)
    measured_xyz = compute_foot_positions(args.urdf.resolve(), q_measured)
    contacts = contact_flags(modes)

    physical_floor, physical_surfaces = load_scene(scene_path, 0.0, math.inf)
    perceived_floor, perceived_surfaces = load_scene(scene_path, terrain_offset, offset_threshold)
    physical_terrain = build_terrain(physical_floor, physical_surfaces, 0.03, smoothing_radius)
    perceived_terrain = build_terrain(perceived_floor, perceived_surfaces, 0.03, smoothing_radius)

    events = touchdown_events(trial_dir, times, contacts, planned_xyz, measured_xyz)
    selected = min(events, key=lambda event: event["time_sec"])
    leg = selected["leg"]
    start_index, final_index = swing_bounds(contacts, leg, selected["index"])
    segment = slice(start_index, final_index + 1)
    segment_times = times[segment]
    phase = (segment_times - segment_times[0]) / (segment_times[-1] - segment_times[0])
    planned = planned_xyz[segment, leg]
    measured = measured_xyz[segment, leg]

    duration = float(segment_times[-1] - segment_times[0])
    scaling = min(1.0, duration / swing_config["swing_time_scale"])
    lift_off_height = float(sample_grid(perceived_terrain, planned[0, 0], planned[0, 1]))
    touch_down_height = float(sample_grid(perceived_terrain, planned[-1, 0], planned[-1, 1]))
    nominal_mid = lift_off_height + scaling * swing_config["swing_height"]
    perceptive_mid = max(lift_off_height, touch_down_height) + scaling * swing_config["swing_height"]
    lift_velocity = scaling * swing_config["lift_off_velocity"]
    touch_velocity = scaling * swing_config["touch_down_velocity"]
    nominal_z, nominal_vz = spline_cpg(
        segment_times,
        segment_times[0],
        segment_times[-1],
        lift_off_height,
        lift_off_height,
        nominal_mid,
        lift_velocity,
        touch_velocity,
    )
    perceptive_z, perceptive_vz = spline_cpg(
        segment_times,
        segment_times[0],
        segment_times[-1],
        lift_off_height,
        touch_down_height,
        perceptive_mid,
        lift_velocity,
        touch_velocity,
    )
    physical_height = sample_grid(physical_terrain, planned[:, 0], planned[:, 1])
    perceived_height = sample_grid(perceived_terrain, planned[:, 0], planned[:, 1])

    configure_publication_style()
    plot_perceptive_pipeline(
        perceived_terrain,
        planned,
        nominal_z,
        perceptive_z,
        lift_off_height,
        touch_down_height,
        selected["leg_name"],
        output_dir,
    )
    write_foothold_csv(events, perceived_terrain, output_dir / "foothold_touchdowns.csv")
    write_swing_csv(
        output_dir / "swing_trajectory.csv",
        segment_times,
        phase,
        planned,
        measured,
        nominal_z,
        nominal_vz,
        perceptive_z,
        perceptive_vz,
        physical_height,
        perceived_height,
    )
    metadata = {
        "trial_dir": str(trial_dir),
        "scenario": config.get("scenario"),
        "scene": str(scene_path),
        "terrain_z_offset_m": terrain_offset,
        "terrain_z_offset_only_below_z_m": offset_threshold,
        "terrain_smoothing_radius_m": smoothing_radius,
        "selected_leg": selected["leg_name"],
        "swing_start_time_sec": float(segment_times[0]),
        "swing_final_time_sec": float(segment_times[-1]),
        "swing_duration_sec": duration,
        "lift_off_height_input_m": lift_off_height,
        "touch_down_height_input_m": touch_down_height,
        "swing_config": swing_config,
        "samples": int(len(segment_times)),
        "method": {
            "planned_and_measured_paths": "Pinocchio FK from tick.csv opt_x and meas_rbd",
            "planner_reference": "Exact CubicSpline/SplineCpg equations from SwingTrajectoryPlanner.cpp",
            "selected_swing": "Earliest recorded lower-surface touchdown",
            "lift_off_height": "Perceived terrain height at the lift-off foothold",
            "pipeline_figure": "Common x-z cross section: planar regions with footholds, nominal spline, modified spline",
        },
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Wrote publication plots and numeric data to {output_dir}")


if __name__ == "__main__":
    main()
