#!/usr/bin/env python3
"""Render presentation figures for the static perceptive-terrain pipeline.

This is an offline, ROS-free mirror of StaticPlanarTerrainPublisher.cpp.  It
parses plane/box geoms from a MuJoCo scene XML, rasterizes their top surfaces,
applies the same finite Gaussian smoothing rule, and draws the planar-region
boundaries and insets consumed by the perceptive controller.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.ndimage import gaussian_filter


NAVY = "#12263A"
BLUE = "#1D70A2"
CYAN = "#36C5D0"
ORANGE = "#F59E0B"
RED = "#EF4444"
MUTED = "#607387"
GRID = "#D8E1E8"
PANEL = "#F7F9FB"

HEIGHT_CMAP = LinearSegmentedColormap.from_list(
    "perceptive_height", ["#E8EEF3", "#B7D8E8", "#54A8C7", "#126782"]
)


@dataclass
class BoxSurface:
    top_center: np.ndarray
    rotation: np.ndarray
    half_x: float
    half_y: float
    true_top_z: float


@dataclass
class TerrainData:
    has_floor: bool
    surfaces: list[BoxSurface]
    x: np.ndarray
    y: np.ndarray
    elevation: np.ndarray
    smooth: np.ndarray
    resolution: float
    smoothing_radius: float


@dataclass
class SwingExample:
    lift_off: np.ndarray
    touch_down: np.ndarray
    lift_off_height: float
    touch_down_height: float
    selected_surface: BoxSurface | None


def configure_fonts() -> None:
    noto_cjk = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if noto_cjk.is_file():
        font_manager.fontManager.addfont(str(noto_cjk))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(noto_cjk)).get_name()
        plt.rcParams["axes.unicode_minus"] = False
        return
    candidates = [
        "Noto Sans CJK KR",
        "Noto Sans KR",
        "NanumGothic",
        "DejaVu Sans",
    ]
    installed = {entry.name for entry in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = next((name for name in candidates if name in installed), "DejaVu Sans")
    plt.rcParams["axes.unicode_minus"] = False


def parse_vector(value: str | None, expected: int, default: tuple[float, ...]) -> np.ndarray:
    if not value:
        return np.asarray(default, dtype=float)
    parsed = np.fromstring(value, sep=" ", dtype=float)
    if parsed.size != expected:
        return np.asarray(default, dtype=float)
    return parsed


def quaternion_to_rotation(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = quat_wxyz / max(np.linalg.norm(quat_wxyz), 1e-12)
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def load_scene(scene_path: Path, z_offset: float, offset_below_z: float) -> tuple[bool, list[BoxSurface]]:
    root = ET.parse(scene_path).getroot()
    has_floor = False
    surfaces: list[BoxSurface] = []

    for geom in root.findall(".//worldbody//geom"):
        geom_type = geom.get("type", "")
        if geom_type == "plane":
            has_floor = True
            continue
        if geom_type != "box" or geom.get("pos") is None or geom.get("size") is None:
            continue

        center = parse_vector(geom.get("pos"), 3, (0.0, 0.0, 0.0))
        size = parse_vector(geom.get("size"), 3, (0.0, 0.0, 0.0))
        quat = parse_vector(geom.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))
        rotation = quaternion_to_rotation(quat)
        top_center = center + rotation @ np.array([0.0, 0.0, size[2]])
        true_top_z = float(top_center[2])
        if true_top_z < offset_below_z:
            top_center[2] += z_offset
        surfaces.append(BoxSurface(top_center, rotation, float(size[0]), float(size[1]), true_top_z))

    return has_floor, surfaces


def surface_corners(surface: BoxSurface, half_x: float | None = None, half_y: float | None = None) -> np.ndarray:
    hx = surface.half_x if half_x is None else half_x
    hy = surface.half_y if half_y is None else half_y
    local = np.array([[hx, hy, 0], [-hx, hy, 0], [-hx, -hy, 0], [hx, -hy, 0]])
    return surface.top_center + local @ surface.rotation.T


def build_terrain(
    has_floor: bool,
    surfaces: list[BoxSurface],
    resolution: float,
    smoothing_radius: float,
) -> TerrainData:
    if surfaces:
        corners = np.vstack([surface_corners(surface)[:, :2] for surface in surfaces])
        min_x, min_y = corners.min(axis=0) - 1.0
        max_x, max_y = corners.max(axis=0) + 1.0
    else:
        min_x, max_x, min_y, max_y = -3.0, 3.0, -2.0, 2.0

    nx = max(2, int(math.ceil((max_x - min_x) / resolution)))
    ny = max(2, int(math.ceil((max_y - min_y) / resolution)))
    x = min_x + (np.arange(nx) + 0.5) * ((max_x - min_x) / nx)
    y = min_y + (np.arange(ny) + 0.5) * ((max_y - min_y) / ny)
    xx, yy = np.meshgrid(x, y)
    elevation = np.zeros_like(xx)
    found = np.full(xx.shape, has_floor or not surfaces, dtype=bool)

    for surface in surfaces:
        projection = surface.rotation[:2, :2]
        determinant = np.linalg.det(projection)
        if abs(determinant) < 1e-9:
            continue
        inv_projection = np.linalg.inv(projection)
        delta = np.stack([xx - surface.top_center[0], yy - surface.top_center[1]], axis=-1)
        local = delta @ inv_projection.T
        mask = (np.abs(local[..., 0]) <= surface.half_x + 1e-6) & (
            np.abs(local[..., 1]) <= surface.half_y + 1e-6
        )
        candidate = (
            surface.top_center[2]
            + surface.rotation[2, 0] * local[..., 0]
            + surface.rotation[2, 1] * local[..., 1]
        )
        replace = mask & ((~found) | (candidate > elevation))
        elevation[replace] = candidate[replace]
        found[replace] = True

    if smoothing_radius <= 0:
        smooth = elevation.copy()
    else:
        radius_cells = max(1, int(math.ceil(smoothing_radius / resolution)))
        sigma_m = max(resolution, smoothing_radius * 0.5)
        sigma_cells = sigma_m / resolution
        smooth = gaussian_filter(
            elevation,
            sigma=sigma_cells,
            mode="constant",
            cval=0.0,
            truncate=radius_cells / sigma_cells,
        )

    return TerrainData(has_floor, surfaces, x, y, elevation, smooth, resolution, smoothing_radius)


def style_axis(ax: plt.Axes) -> None:
    ax.set_facecolor(PANEL)
    ax.grid(color=GRID, linewidth=0.7, alpha=0.7)
    ax.tick_params(colors=MUTED, labelsize=9)
    for spine in ax.spines.values():
        spine.set_color(GRID)


def add_title(fig: plt.Figure, title: str, subtitle: str) -> None:
    fig.text(0.055, 0.945, title, fontsize=25, fontweight="bold", color=NAVY, va="top")
    fig.text(0.055, 0.902, subtitle, fontsize=11, color=MUTED, va="top")


def add_height_map(ax: plt.Axes, data: TerrainData, values: np.ndarray, title: str, subtitle: str) -> None:
    extent = [data.x[0], data.x[-1], data.y[0], data.y[-1]]
    image = ax.imshow(values, origin="lower", extent=extent, cmap=HEIGHT_CMAP, aspect="equal")
    ax.set_title(title, loc="left", fontsize=15, fontweight="bold", color=NAVY, pad=14)
    ax.text(0.0, 1.015, subtitle, transform=ax.transAxes, fontsize=9, color=MUTED, va="bottom")
    ax.set_xlabel("x [m]", color=MUTED)
    ax.set_ylabel("y [m]", color=MUTED)
    style_axis(ax)
    colorbar = plt.colorbar(image, ax=ax, fraction=0.038, pad=0.025)
    colorbar.set_label("height [m]", color=MUTED)
    colorbar.ax.tick_params(labelsize=8, colors=MUTED)


def add_region_overlay(ax: plt.Axes, data: TerrainData, show_labels: bool = True) -> None:
    min_x, max_x = data.x[0], data.x[-1]
    min_y, max_y = data.y[0], data.y[-1]
    if data.has_floor or not data.surfaces:
        floor = np.array([[max_x, max_y], [min_x, max_y], [min_x, min_y], [max_x, min_y]])
        ax.add_patch(Polygon(floor, fill=False, edgecolor=BLUE, linewidth=1.8, alpha=0.55))

    for index, surface in enumerate(data.surfaces, start=1):
        boundary = surface_corners(surface)[:, :2]
        inset_margin = max(0.005, min(0.02, surface.half_x * 0.2, surface.half_y * 0.2))
        inset_x = max(surface.half_x - inset_margin, surface.half_x * 0.5)
        inset_y = max(surface.half_y - inset_margin, surface.half_y * 0.5)
        inset = surface_corners(surface, inset_x, inset_y)[:, :2]
        ax.add_patch(Polygon(boundary, fill=False, edgecolor=ORANGE, linewidth=3.0, joinstyle="round"))
        ax.add_patch(
            Polygon(inset, fill=False, edgecolor=RED, linewidth=2.0, linestyle=(0, (5, 3)), joinstyle="round")
        )
        if show_labels:
            ax.text(
                surface.top_center[0],
                surface.top_center[1],
                f"region {index}\nz={surface.top_center[2]:.2f} m",
                ha="center",
                va="center",
                fontsize=9,
                color=NAVY,
                fontweight="bold",
                bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": "none", "alpha": 0.82},
            )


def make_swing_example(data: TerrainData) -> SwingExample:
    """Choose a deterministic one-leg example from the first two x-ordered regions."""
    if not data.surfaces:
        lift_off = np.array([-0.35, 0.25])
        touch_down = np.array([0.35, 0.25])
        return SwingExample(lift_off, touch_down, 0.0, 0.0, None)

    ordered = sorted(data.surfaces, key=lambda surface: surface.top_center[0])
    source = ordered[0]
    target = ordered[1] if len(ordered) > 1 else ordered[0]
    lateral = min(0.28, source.half_y * 0.35, target.half_y * 0.35)

    if target is source:
        source_local_x = -0.35 * source.half_x
        target_local_x = 0.35 * source.half_x
    else:
        source_local_x = 0.58 * source.half_x
        target_local_x = -0.65 * target.half_x

    lift_off_world = source.top_center + source.rotation @ np.array([source_local_x, lateral, 0.0])
    touch_down_world = target.top_center + target.rotation @ np.array([target_local_x, lateral, 0.0])
    return SwingExample(
        lift_off_world[:2],
        touch_down_world[:2],
        float(lift_off_world[2]),
        float(touch_down_world[2]),
        target,
    )


def cubic_segment(
    time: np.ndarray,
    start_time: float,
    start_position: float,
    start_velocity: float,
    final_time: float,
    final_position: float,
    final_velocity: float,
) -> np.ndarray:
    """Match the CubicSpline coefficients used by SwingTrajectoryPlanner."""
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
    return c3 * normalized**3 + c2 * normalized**2 + c1 * normalized + c0


def swing_height_spline(
    phase: np.ndarray,
    lift_off_height: float,
    touch_down_height: float,
    mid_height: float,
    duration: float,
    lift_off_velocity: float,
    touch_down_velocity: float,
) -> np.ndarray:
    time = phase * duration
    mid_time = duration * 0.5
    height = np.empty_like(phase)
    left = time < mid_time
    height[left] = cubic_segment(
        time[left], 0.0, lift_off_height, lift_off_velocity, mid_time, mid_height, 0.0
    )
    height[~left] = cubic_segment(
        time[~left], mid_time, mid_height, 0.0, duration, touch_down_height, touch_down_velocity
    )
    return height


def render_scene_geometry(data: TerrainData, output: Path) -> None:
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    add_title(fig, "01  Scene geometry → top surfaces", "MuJoCo를 실행하지 않고 XML의 plane / box geom만 읽습니다.")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_position([0.07, 0.09, 0.86, 0.75])
    xx, yy = np.meshgrid(data.x[::3], data.y[::3])
    ax.plot_surface(xx, yy, np.zeros_like(xx), color="#E7EDF2", alpha=0.55, linewidth=0)
    for index, surface in enumerate(data.surfaces, start=1):
        corners = surface_corners(surface)
        ax.add_collection3d(
            Poly3DCollection(
                [corners], facecolor=CYAN, edgecolor=NAVY, linewidth=1.5, alpha=0.85
            )
        )
        ax.text(*surface.top_center, f"  top {index}: {surface.top_center[2]:.2f} m", color=NAVY, fontsize=10)
    ax.set_xlabel("x [m]", labelpad=10)
    ax.set_ylabel("y [m]", labelpad=10)
    ax.set_zlabel("height [m]", labelpad=8)
    ax.set_xlim(data.x[0], data.x[-1])
    ax.set_ylim(data.y[0], data.y[-1])
    max_z = max([surface.top_center[2] for surface in data.surfaces] + [0.1])
    ax.set_zlim(0, max_z * 1.8)
    ax.view_init(elev=28, azim=-60)
    ax.set_box_aspect((data.x[-1] - data.x[0], data.y[-1] - data.y[0], max_z * 3.5))
    ax.grid(color=GRID, alpha=0.6)
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def render_elevation(data: TerrainData, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 9), facecolor="white")
    fig.subplots_adjust(left=0.16, right=0.84, bottom=0.10, top=0.82)
    add_title(fig, "02  Rasterization → elevation", f"해상도 {data.resolution:.3f} m 셀마다 가장 높은 top surface를 기록합니다.")
    add_height_map(ax, data, data.elevation, "Raw elevation grid", "StaticPlanarTerrainPublisher: elevation layer")
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def render_smoothing(data: TerrainData, output: Path) -> None:
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    add_title(
        fig,
        "03  Gaussian smoothing → smooth_planar",
        f"radius={data.smoothing_radius:.2f} m · 가장자리의 불연속을 완만한 높이/법선 신호로 바꿉니다.",
    )
    grid = fig.add_gridspec(2, 2, left=0.065, right=0.94, bottom=0.09, top=0.82, height_ratios=[3.1, 1.3], hspace=0.35)
    raw_ax = fig.add_subplot(grid[0, 0])
    smooth_ax = fig.add_subplot(grid[0, 1])
    profile_ax = fig.add_subplot(grid[1, :])
    add_height_map(raw_ax, data, data.elevation, "Before", "elevation")
    add_height_map(smooth_ax, data, data.smooth, "After", "smooth_planar")
    row = int(np.argmin(np.abs(data.y)))
    profile_ax.plot(data.x, data.elevation[row], color=MUTED, linewidth=2, label="elevation")
    profile_ax.plot(data.x, data.smooth[row], color=BLUE, linewidth=3, label="smooth_planar")
    profile_ax.set_title("Center-line profile (y ≈ 0)", loc="left", fontsize=13, fontweight="bold", color=NAVY)
    profile_ax.set_xlabel("x [m]", color=MUTED)
    profile_ax.set_ylabel("height [m]", color=MUTED)
    style_axis(profile_ax)
    profile_ax.legend(frameon=False, ncol=2, loc="upper right")
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def render_regions(data: TerrainData, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 9), facecolor="white")
    fig.subplots_adjust(left=0.13, right=0.86, bottom=0.10, top=0.82)
    add_title(
        fig,
        "04  PlanarTerrain packaging → boundaries & insets",
        "box top마다 평면 region을 만들고 경계(주황)와 안전 inset(빨강 점선)을 함께 전달합니다.",
    )
    add_height_map(ax, data, data.smooth, "Controller terrain representation", "smooth_planar + planar regions")
    add_region_overlay(ax, data)
    ax.plot([], [], color=ORANGE, linewidth=3, label="planar-region boundary")
    ax.plot([], [], color=RED, linewidth=2, linestyle=(0, (5, 3)), label="inset")
    ax.legend(frameon=False, loc="upper right", fontsize=10)
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def render_foothold_selection(data: TerrainData, output: Path) -> SwingExample:
    example = make_swing_example(data)
    fig, ax = plt.subplots(figsize=(16, 9), facecolor="white")
    fig.subplots_adjust(left=0.13, right=0.86, bottom=0.10, top=0.82)
    add_title(
        fig,
        "05  Terrain data → foothold selection",
        "nominal foothold를 가장 적합한 planar region에 투영하고 착지점을 확정합니다.",
    )
    add_height_map(ax, data, data.smooth, "Received PlanarTerrain", "smooth_planar + planar-region boundaries")
    add_region_overlay(ax, data, show_labels=False)

    if example.selected_surface is not None:
        selected_boundary = surface_corners(example.selected_surface)[:, :2]
        ax.add_patch(
            Polygon(selected_boundary, closed=True, facecolor=ORANGE, edgecolor="none", alpha=0.12, zorder=3)
        )

    ax.annotate(
        "",
        xy=example.touch_down,
        xytext=example.lift_off,
        arrowprops={"arrowstyle": "-|>", "color": NAVY, "lw": 2.4, "mutation_scale": 18},
        zorder=7,
    )
    ax.scatter(
        *example.lift_off,
        s=160,
        marker="o",
        facecolor="white",
        edgecolor=NAVY,
        linewidth=2.8,
        zorder=8,
        label="lift-off foothold",
    )
    ax.scatter(
        *example.touch_down,
        s=260,
        marker="*",
        facecolor=ORANGE,
        edgecolor="white",
        linewidth=1.6,
        zorder=9,
        label="selected touchdown foothold",
    )
    ax.text(
        example.lift_off[0] - 0.08,
        example.lift_off[1] + 0.16,
        f"lift-off\nz={example.lift_off_height:.2f} m",
        ha="right",
        va="bottom",
        fontsize=10,
        color=NAVY,
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "none", "alpha": 0.90},
        zorder=10,
    )
    ax.text(
        example.touch_down[0] + 0.08,
        example.touch_down[1] + 0.16,
        f"projected foothold\nz={example.touch_down_height:.2f} m",
        ha="left",
        va="bottom",
        fontsize=10,
        color=NAVY,
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "none", "alpha": 0.90},
        zorder=10,
    )
    midpoint = 0.5 * (example.lift_off + example.touch_down)
    ax.text(
        midpoint[0],
        midpoint[1] - 0.16,
        "next swing",
        ha="center",
        va="top",
        fontsize=10,
        color=NAVY,
        fontweight="bold",
        zorder=10,
    )
    ax.legend(frameon=False, loc="upper right", fontsize=10)
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)
    return example


def render_swing_adaptation(
    data: TerrainData,
    example: SwingExample,
    output: Path,
    swing_height: float,
    swing_duration: float,
    lift_off_velocity: float,
    touch_down_velocity: float,
) -> None:
    scaling = min(1.0, swing_duration / 0.15)
    phase = np.linspace(0.0, 1.0, 240)
    path_x = example.lift_off[0] + phase * (example.touch_down[0] - example.lift_off[0])
    nominal_mid = example.lift_off_height + scaling * swing_height
    perceptive_mid = max(example.lift_off_height, example.touch_down_height) + scaling * swing_height
    nominal_z = swing_height_spline(
        phase,
        example.lift_off_height,
        example.lift_off_height,
        nominal_mid,
        swing_duration,
        lift_off_velocity,
        touch_down_velocity,
    )
    perceptive_z = swing_height_spline(
        phase,
        example.lift_off_height,
        example.touch_down_height,
        perceptive_mid,
        swing_duration,
        lift_off_velocity,
        touch_down_velocity,
    )

    fig, ax = plt.subplots(figsize=(16, 9), facecolor="white")
    fig.subplots_adjust(left=0.09, right=0.95, bottom=0.12, top=0.80)
    add_title(
        fig,
        "06  Foothold height → perceptive swing trajectory",
        "lift-off / touchdown 높이를 terrain projection에서 가져와 SplineCpg의 끝점과 중간 높이를 수정합니다.",
    )
    row = int(np.argmin(np.abs(data.y - example.lift_off[1])))
    terrain_x = data.x
    terrain_z = data.elevation[row]
    ax.fill_between(terrain_x, -0.04, terrain_z, color="#DCE6EC", alpha=1.0, step="mid", label="terrain data")
    ax.plot(terrain_x, terrain_z, color=MUTED, linewidth=2.2, drawstyle="steps-mid", zorder=3)
    ax.plot(
        path_x,
        nominal_z,
        color=MUTED,
        linewidth=2.7,
        linestyle=(0, (7, 5)),
        label="nominal: same-height touchdown",
        zorder=5,
    )
    ax.plot(
        path_x,
        perceptive_z,
        color=BLUE,
        linewidth=5.0,
        label="perceptive: terrain-aware touchdown",
        zorder=6,
    )

    sample_indices = np.linspace(0, len(phase) - 1, 7, dtype=int)
    ax.scatter(
        path_x[sample_indices],
        perceptive_z[sample_indices],
        s=46,
        facecolor="white",
        edgecolor=BLUE,
        linewidth=2.0,
        zorder=7,
    )
    ax.scatter(
        *[example.lift_off[0], example.lift_off_height],
        s=170,
        marker="o",
        facecolor="white",
        edgecolor=NAVY,
        linewidth=2.8,
        zorder=8,
    )
    ax.scatter(
        *[example.touch_down[0], example.touch_down_height],
        s=280,
        marker="*",
        facecolor=ORANGE,
        edgecolor="white",
        linewidth=1.6,
        zorder=9,
    )
    ax.annotate(
        f"selected foothold\nz={example.touch_down_height:.2f} m",
        xy=(example.touch_down[0], example.touch_down_height),
        xytext=(example.touch_down[0] + 0.18, example.touch_down_height + 0.055),
        color=NAVY,
        fontsize=11,
        fontweight="bold",
        arrowprops={"arrowstyle": "->", "color": ORANGE, "lw": 1.8},
        bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "none", "alpha": 0.92},
        zorder=10,
    )
    ax.annotate(
        f"mid height = max(z_lift-off, z_touchdown) + {swing_height:.2f} m",
        xy=(path_x[len(path_x) // 2], perceptive_mid),
        xytext=(path_x[len(path_x) // 2] - 0.05, perceptive_mid + 0.055),
        ha="center",
        color=BLUE,
        fontsize=10,
        fontweight="bold",
        arrowprops={"arrowstyle": "->", "color": BLUE, "lw": 1.5},
        zorder=10,
    )
    landing_delta = example.lift_off_height - example.touch_down_height
    if abs(landing_delta) > 1e-6:
        ax.vlines(
            example.touch_down[0],
            min(example.lift_off_height, example.touch_down_height),
            max(example.lift_off_height, example.touch_down_height),
            color=RED,
            linewidth=1.8,
            linestyle=(0, (3, 3)),
            zorder=4,
        )
        ax.text(
            example.touch_down[0] + 0.025,
            0.5 * (example.lift_off_height + example.touch_down_height),
            f"Δz={landing_delta:+.2f} m",
            color=RED,
            fontsize=10,
            fontweight="bold",
            va="center",
        )

    x_margin = max(0.22, 0.22 * abs(example.touch_down[0] - example.lift_off[0]))
    ax.set_xlim(example.lift_off[0] - x_margin, example.touch_down[0] + x_margin)
    max_height = max(nominal_z.max(), perceptive_z.max())
    ax.set_ylim(min(-0.02, terrain_z.min() - 0.02), max_height + 0.10)
    ax.set_xlabel("forward position x [m]", color=MUTED)
    ax.set_ylabel("foot height z [m]", color=MUTED)
    ax.set_title("One-leg swing over the received terrain", loc="left", fontsize=15, fontweight="bold", color=NAVY)
    style_axis(ax)
    ax.legend(frameon=False, loc="upper right", fontsize=10)
    ax.text(
        0.0,
        -0.14,
        "Note: horizontal x interpolation is illustrative; the controller's SwingTrajectoryPlanner generates the z constraint.",
        transform=ax.transAxes,
        fontsize=9,
        color=MUTED,
    )
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def render_overview(data: TerrainData, output: Path, scene_name: str) -> None:
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    add_title(
        fig,
        "Perceptive terrain preprocessing — offline view",
        f"{scene_name} · MuJoCo / ROS 없이 현재 static publisher의 데이터 흐름을 재현",
    )
    grid = fig.add_gridspec(2, 4, left=0.045, right=0.965, bottom=0.13, top=0.80, wspace=0.38, hspace=0.46)
    axes = [fig.add_subplot(grid[0, :2]), fig.add_subplot(grid[0, 2:]), fig.add_subplot(grid[1, :2]), fig.add_subplot(grid[1, 2:])]

    add_height_map(axes[0], data, data.elevation, "1  Surface rasterization", "XML plane / box → elevation")
    add_height_map(axes[1], data, data.smooth, "2  Gaussian smoothing", "elevation → smooth_planar")
    add_height_map(axes[2], data, data.smooth, "3  Region representation", "boundary + inset")
    add_region_overlay(axes[2], data, show_labels=False)

    profile_ax = axes[3]
    row = int(np.argmin(np.abs(data.y)))
    profile_ax.plot(data.x, data.elevation[row], color=MUTED, linewidth=2, label="elevation")
    profile_ax.plot(data.x, data.smooth[row], color=BLUE, linewidth=3, label="smooth_planar")
    for surface in data.surfaces:
        profile_ax.scatter(surface.top_center[0], surface.top_center[2], s=65, color=ORANGE, edgecolor="white", zorder=5)
    profile_ax.set_title("4  Features consumed by MPC", loc="left", fontsize=15, fontweight="bold", color=NAVY, pad=14)
    profile_ax.text(
        0.0,
        1.015,
        "terrain height / normal · foothold projection · swing touchdown height",
        transform=profile_ax.transAxes,
        fontsize=9,
        color=MUTED,
        va="bottom",
    )
    profile_ax.set_xlabel("x [m]", color=MUTED)
    profile_ax.set_ylabel("height [m]", color=MUTED)
    style_axis(profile_ax)
    profile_ax.legend(frameon=False, fontsize=9, loc="upper right")

    labels = ["XML geometry", "elevation", "smooth_planar", "PlanarTerrain → MPC"]
    positions = np.linspace(0.12, 0.88, len(labels))
    for position, label in zip(positions, labels):
        fig.text(position, 0.055, label, ha="center", va="center", fontsize=11, color=NAVY, fontweight="bold")
    for left, right in zip(positions[:-1], positions[1:]):
        fig.add_artist(
            matplotlib.patches.FancyArrowPatch(
                (left + 0.065, 0.055),
                (right - 0.075, 0.055),
                transform=fig.transFigure,
                arrowstyle="-|>",
                mutation_scale=13,
                linewidth=1.8,
                color=CYAN,
            )
        )

    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True, help="MuJoCo scene XML containing plane/box geoms")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for the five PNG figures")
    parser.add_argument("--resolution", type=float, default=0.03, help="Grid resolution in metres (default: 0.03)")
    parser.add_argument("--smoothing-radius", type=float, default=0.12, help="Gaussian support radius in metres")
    parser.add_argument("--swing-height", type=float, default=0.08, help="Go2 swing apex offset in metres")
    parser.add_argument("--swing-duration", type=float, default=0.30, help="Illustrated swing duration in seconds")
    parser.add_argument("--lift-off-velocity", type=float, default=0.05, help="Vertical lift-off velocity in m/s")
    parser.add_argument("--touch-down-velocity", type=float, default=-0.10, help="Vertical touchdown velocity in m/s")
    parser.add_argument("--terrain-z-offset", type=float, default=0.0, help="Perception-only box top-height offset")
    parser.add_argument(
        "--terrain-z-offset-only-below-z",
        type=float,
        default=float("inf"),
        help="Apply the perception offset only to true box tops below this height",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scene = args.scene.resolve()
    if not scene.is_file():
        raise FileNotFoundError(scene)
    if args.resolution <= 0:
        raise ValueError("--resolution must be positive")
    if args.smoothing_radius < 0:
        raise ValueError("--smoothing-radius cannot be negative")
    if args.swing_height < 0:
        raise ValueError("--swing-height cannot be negative")
    if args.swing_duration <= 0:
        raise ValueError("--swing-duration must be positive")

    configure_fonts()
    has_floor, surfaces = load_scene(scene, args.terrain_z_offset, args.terrain_z_offset_only_below_z)
    terrain = build_terrain(has_floor, surfaces, args.resolution, args.smoothing_radius)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    render_scene_geometry(terrain, output / "01_scene_geometry.png")
    render_elevation(terrain, output / "02_elevation.png")
    render_smoothing(terrain, output / "03_smooth_planar.png")
    render_regions(terrain, output / "04_planar_regions.png")
    example = render_foothold_selection(terrain, output / "05_foothold_selection.png")
    render_swing_adaptation(
        terrain,
        example,
        output / "06_swing_trajectory_adaptation.png",
        args.swing_height,
        args.swing_duration,
        args.lift_off_velocity,
        args.touch_down_velocity,
    )
    render_overview(terrain, output / "perceptive_preprocessing_overview.png", scene.name)

    print(f"Rendered 7 figures in {output}")


if __name__ == "__main__":
    main()
