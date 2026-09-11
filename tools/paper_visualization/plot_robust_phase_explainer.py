#!/usr/bin/env python3
"""Create a publication schematic of nominal and robust contact switching.

The numeric labels are read from the Go2 task.info robustPhase and sqp blocks.
The diagram mirrors the implementation in PerceptiveLeggedReferenceManager:

  T_robust = P * dt_mpc
  g(t_a) = +d,  g(t_b) = -d
  -v_max <= g_dot <= 0

An optional dashed branch illustrates the measured-contact stance splice and
same-cycle MPC replan used when robustPhase.enable_splice is enabled.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, FancyArrowPatch, FancyBboxPatch
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_TASK_INFO = REPOSITORY_ROOT / "descriptions/unitree/go2_description/config/ocs2/task.info"

GRAY = "#6F6F6F"
LIGHT_GRAY = "#F2F2F2"
BLUE = "#5B84C4"
LIGHT_BLUE = "#E7EFFA"
GREEN = "#73AD5A"
LIGHT_GREEN = "#E8F3E2"
BLACK = "#222222"
GRID = "#D8D8D8"
CONTENT_SCALE = 1.5
PHASE_FONT_SIZE = 10 * CONTENT_SCALE
NODE_RADIUS_Y = 0.055
ROBUST_FIRST_X = 0.405
ROBUST_LAST_X = 0.595
NODE_STEP_X = 0.075
CONTACT_EVENT_U = 0.62


def configure_style() -> None:
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
            "font.size": PHASE_FONT_SIZE,
            "axes.titlesize": 11 * CONTENT_SCALE,
            "mathtext.fontset": "stix",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.04,
        }
    )


def extract_block(text: str, block: str) -> str:
    match = re.search(
        rf"^\s*{re.escape(block)}\s*$\s*^\s*\{{\s*$\n(.*?)^\s*\}}\s*$",
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
    if match is None:
        raise RuntimeError(f"Could not find '{block}' block")
    return match.group(1)


def read_value(text: str, key: str, block: str) -> str:
    body = extract_block(text, block)
    match = re.search(rf"^\s*{re.escape(key)}\s+([^;\s]+)", body, flags=re.MULTILINE)
    if match is None:
        raise RuntimeError(f"Could not find '{block}.{key}'")
    return match.group(1)


def load_settings(task_info: Path) -> dict[str, float | int | bool | str]:
    text = task_info.read_text(encoding="utf-8")
    dt = float(read_value(text, "dt", "sqp"))
    p = int(read_value(text, "P", "robustPhase"))
    d = float(read_value(text, "d", "robustPhase"))
    v_max = float(read_value(text, "v_max", "robustPhase"))
    offset = float(read_value(text, "foot_frame_offset", "robustPhase"))
    terrain_source = read_value(text, "terrain_source", "robustPhase")
    splice_text = read_value(text, "enable_splice", "robustPhase").lower()
    enable_splice = splice_text in {"true", "1", "yes", "on"}
    return {
        "P": p,
        "dt_mpc": dt,
        "T_robust": p * dt,
        "d": d,
        "v_max": v_max,
        "foot_frame_offset": offset,
        "terrain_source": terrain_source,
        "enable_splice": enable_splice,
        "minimum_traversal_time": 2.0 * d / v_max,
    }


def draw_node(ax: plt.Axes, x: float, y: float, *, edge: str, face: str, size: float = 165) -> None:
    ax.scatter(
        x,
        y,
        s=size * CONTENT_SCALE**2,
        facecolor=face,
        edgecolor=edge,
        linewidth=1.5 * CONTENT_SCALE,
        zorder=6,
    )


def draw_mode_box(
    ax: plt.Axes,
    x0: float,
    y0: float,
    width: float,
    height: float,
    *,
    edge: str,
    face: str,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (x0, y0),
            width,
            height,
            boxstyle="round,pad=0.008,rounding_size=0.018",
            linewidth=1.4 * CONTENT_SCALE,
            edgecolor=edge,
            facecolor=face,
            zorder=0,
        )
    )


def draw_nominal_panel(ax: plt.Axes) -> None:
    ax.set_title("(a)  Nominal contact switch", loc="left", fontweight="bold", pad=4)
    ax.set_xlim(0.07, 0.93)
    ax.set_ylim(0.25, 0.96)
    ax.axis("off")

    node_y = 0.48 - NODE_RADIUS_Y
    draw_mode_box(ax, 0.10, 0.32, 0.45, 0.31, edge="#A0A0A0", face="#FAFAFA")
    ax.text(0.325, 0.69, "swing phase", ha="center", va="bottom", fontsize=PHASE_FONT_SIZE)
    ax.text(0.72, 0.69, "stance phase", ha="center", va="bottom", fontsize=PHASE_FONT_SIZE)

    pre_nodes = (0.16, 0.235, 0.31, 0.50)
    for index, x in enumerate(pre_nodes):
        draw_node(ax, float(x), node_y, edge=GRAY, face="#FAFAFA")
        if index < 3:
            ax.text(x, node_y + 0.115, f"${index}$", ha="center", fontsize=PHASE_FONT_SIZE)
    ax.text(0.405, node_y, "$\cdots$", ha="center", va="center", fontsize=PHASE_FONT_SIZE)

    post_nodes = (0.62, 0.69, 0.82)
    for x in post_nodes:
        draw_node(ax, x, node_y, edge=BLUE, face=LIGHT_BLUE)
    ax.text(
        post_nodes[-1], node_y + 0.115, "$N-1$", ha="center", fontsize=PHASE_FONT_SIZE, color=BLUE
    )
    ax.text(
        0.5 * (post_nodes[1] + post_nodes[-1]),
        node_y,
        "$\cdots$",
        color=BLUE,
        ha="center",
        va="center",
        fontsize=PHASE_FONT_SIZE,
    )

    last_swing_x = pre_nodes[-1]
    first_stance_x = post_nodes[0]
    switch_mid_x = 0.5 * (last_swing_x + first_stance_x)
    ax.plot(
        [last_swing_x, last_swing_x, switch_mid_x, first_stance_x],
        [node_y + 0.055, 0.78, 0.84, 0.84],
        color=BLUE,
        linewidth=1.4 * CONTENT_SCALE,
        solid_capstyle="round",
    )
    ax.add_patch(
        FancyArrowPatch(
            (first_stance_x, 0.84),
            (first_stance_x, node_y + 0.06),
            arrowstyle="-|>",
            mutation_scale=11 * CONTENT_SCALE,
            linewidth=1.4 * CONTENT_SCALE,
            color=BLUE,
        )
    )
    ax.text(switch_mid_x, 0.88, "switch", color=BLUE, ha="center", fontsize=PHASE_FONT_SIZE)


def robust_node_positions(p: int, x0: float, x1: float) -> np.ndarray:
    del p
    return np.array((x0, x0 + NODE_STEP_X, x1))


def draw_robust_panel(ax: plt.Axes, settings: dict, show_replan: bool) -> None:
    p = int(settings["P"])
    ax.set_title(
        "(b)  Contact switch with robust phase",
        loc="left",
        fontweight="bold",
        pad=4,
        bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.15},
        zorder=10,
    )
    ax.set_xlim(0.07, 0.93)
    ax.set_ylim(0.0, 0.88)
    ax.axis("off")

    node_y = 0.58 - NODE_RADIUS_Y
    x_a, x_b = ROBUST_FIRST_X, ROBUST_LAST_X
    draw_mode_box(ax, 0.10, 0.47, 0.27, 0.22, edge="#A0A0A0", face="#FAFAFA")
    draw_mode_box(ax, x_a - 0.015, 0.45, x_b - x_a + 0.03, 0.26, edge=GREEN, face="#FAFAFA")
    ax.text(0.235, 0.74, "swing phase", ha="center", fontsize=PHASE_FONT_SIZE)
    ax.text(
        0.5 * (x_a + x_b),
        0.76,
        "robust phase",
        ha="center",
        color=GREEN,
        fontsize=PHASE_FONT_SIZE,
        fontweight="bold",
    )

    pre_nodes = (0.16, 0.235, 0.31)
    for index, x in enumerate(pre_nodes):
        draw_node(ax, x, node_y, edge=GRAY, face="#FAFAFA", size=145)
        ax.text(x, node_y + 0.105, f"${index}$", ha="center", fontsize=PHASE_FONT_SIZE)
    ax.text(0.355, node_y, "$\cdots$", ha="center", va="center", fontsize=PHASE_FONT_SIZE)

    robust_x = robust_node_positions(p, x_a, x_b)
    for x in robust_x:
        draw_node(ax, float(x), node_y, edge=GREEN, face=LIGHT_GREEN, size=120)
    ax.text(
        robust_x[0],
        node_y + 0.105,
        "$\\mathcal{P}_0$",
        color=GREEN,
        ha="center",
        fontsize=PHASE_FONT_SIZE,
    )
    ax.text(
        0.5 * (robust_x[1] + robust_x[2]),
        node_y,
        "$\\cdots$",
        color=GREEN,
        ha="center",
        va="center",
        fontsize=PHASE_FONT_SIZE,
    )
    ax.text(
        robust_x[-1] - 0.012,
        node_y + 0.105,
        "$\\mathcal{P}_{M\\!-\\!1}$",
        color=GREEN,
        ha="center",
        fontsize=PHASE_FONT_SIZE,
    )
    ax.text(
        0.5 * (robust_x[0] + robust_x[-1]),
        node_y + 0.105,
        "$\\cdots$",
        color=GREEN,
        ha="center",
        fontsize=PHASE_FONT_SIZE,
    )
    # A smooth monotone guard path satisfying the two robust boundary targets.
    u = np.linspace(0.0, 1.0, 200)
    smoothstep = 3.0 * u**2 - 2.0 * u**3
    guard_normalized = 1.0 - 2.0 * smoothstep
    guard_x = x_a + (x_b - x_a) * u
    guard_center_y = 0.19
    guard_amplitude = 0.095
    guard_y = guard_center_y + guard_amplitude * guard_normalized
    ax.plot(
        [x_a, x_b],
        [guard_center_y, guard_center_y],
        color=GRID,
        linestyle=(0, (4, 3)),
        linewidth=0.8 * CONTENT_SCALE,
        zorder=1,
    )
    ax.plot(guard_x, guard_y, color=GREEN, linewidth=2.0 * CONTENT_SCALE, zorder=3)
    ax.plot(
        [x_a, x_a],
        [0.45, guard_center_y + guard_amplitude],
        color=GREEN,
        linewidth=1.0 * CONTENT_SCALE,
    )
    ax.plot(
        [x_b, x_b],
        [0.45, guard_center_y - guard_amplitude],
        color=GREEN,
        linewidth=1.0 * CONTENT_SCALE,
    )
    ax.text(
        x_a - 0.012,
        guard_center_y + guard_amplitude,
        "$g(x_{\\mathcal{P}_0})=+d$",
        ha="right",
        va="center",
        color=GREEN,
        fontsize=PHASE_FONT_SIZE,
    )
    ax.text(
        x_b + 0.012,
        guard_center_y - guard_amplitude,
        "$g(x_{\\mathcal{P}_{M\\!-\\!1}})=-d$",
        ha="left",
        va="center",
        color=GREEN,
        fontsize=PHASE_FONT_SIZE,
    )
    ax.text(
        x_a - 0.012,
        guard_center_y,
        "$0$",
        ha="right",
        va="center",
        color=GRAY,
        fontsize=PHASE_FONT_SIZE,
    )

    event_smoothstep = 3.0 * CONTACT_EVENT_U**2 - 2.0 * CONTACT_EVENT_U**3
    event_x = x_a + (x_b - x_a) * CONTACT_EVENT_U
    event_y = guard_center_y + guard_amplitude * (1.0 - 2.0 * event_smoothstep)
    ax.scatter(
        event_x,
        event_y,
        s=36 * CONTENT_SCALE**2,
        color=BLUE,
        edgecolor="white",
        linewidth=0.7 * CONTENT_SCALE,
        zorder=7,
    )
    ax.text(
        event_x,
        event_y + 0.115,
        "contact detected",
        ha="center",
        va="bottom",
        color=BLUE,
        fontsize=PHASE_FONT_SIZE,
    )

    if show_replan:
        stance_y = node_y
        stance_x = (0.68, 0.75, 0.87)
        ax.add_patch(
            FancyArrowPatch(
                (event_x + 0.01, event_y + 0.01),
                (stance_x[0] - 0.018, stance_y - 0.045),
                arrowstyle="-|>",
                mutation_scale=11 * CONTENT_SCALE,
                connectionstyle="arc3,rad=0.34",
                linestyle=(0, (3, 3)),
                linewidth=1.5 * CONTENT_SCALE,
                color=BLUE,
            )
        )
        for x in stance_x:
            draw_node(ax, x, stance_y, edge=BLUE, face=LIGHT_BLUE, size=150)
        ax.text(
            0.5 * (stance_x[1] + stance_x[-1]),
            stance_y,
            "$\cdots$",
            color=BLUE,
            ha="center",
            va="center",
            fontsize=PHASE_FONT_SIZE,
        )
        ax.text(0.775, 0.74, "stance phase", ha="center", color=BLACK, fontsize=PHASE_FONT_SIZE)
        ax.text(
            0.79,
            0.47 - NODE_RADIUS_Y,
            "contact-triggered replan",
            ha="center",
            color=BLUE,
            fontsize=PHASE_FONT_SIZE,
        )
        ax.text(
            stance_x[-1],
            stance_y + 0.105,
            "$N-1$",
            ha="center",
            color=BLUE,
            fontsize=PHASE_FONT_SIZE,
        )
    else:
        stance_x = (0.68, 0.75, 0.87)
        for x in stance_x:
            draw_node(ax, x, node_y, edge=BLUE, face=LIGHT_BLUE, size=150)
        ax.text(
            0.5 * (stance_x[1] + stance_x[-1]),
            node_y,
            "$\cdots$",
            color=BLUE,
            ha="center",
            va="center",
            fontsize=PHASE_FONT_SIZE,
        )
        ax.text(0.775, 0.74, "stance phase", ha="center", color=BLACK, fontsize=PHASE_FONT_SIZE)
        ax.text(
            stance_x[-1],
            node_y + 0.105,
            "$N-1$",
            ha="center",
            color=BLUE,
            fontsize=PHASE_FONT_SIZE,
        )


def save_figure(fig: plt.Figure, output_base: Path) -> None:
    fig.savefig(output_base.with_suffix(".pdf"))
    fig.savefig(output_base.with_suffix(".svg"))
    fig.savefig(output_base.with_suffix(".png"), dpi=300)
    plt.close(fig)


def draw_robust_expansion_arrows(fig: plt.Figure, nominal_ax: plt.Axes, robust_ax: plt.Axes) -> None:
    """Expand the nominal terminal swing node into the robust-phase interval."""
    start = (0.50, 0.48 - 2.0 * NODE_RADIUS_Y)
    robust_top_y = 0.715
    for target_x, curvature in (
        (ROBUST_FIRST_X - 0.015, 0.10),
        (ROBUST_LAST_X + 0.015, -0.10),
    ):
        arrow = ConnectionPatch(
            xyA=start,
            coordsA=nominal_ax.transData,
            xyB=(target_x, robust_top_y),
            coordsB=robust_ax.transData,
            arrowstyle="-|>",
            mutation_scale=10 * CONTENT_SCALE,
            connectionstyle=f"arc3,rad={curvature}",
            linewidth=1.35 * CONTENT_SCALE,
            color=GREEN,
            shrinkA=2,
            shrinkB=2,
            clip_on=False,
            zorder=3,
        )
        fig.add_artist(arrow)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-info", type=Path, default=DEFAULT_TASK_INFO)
    parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR / "paper_output")
    parser.add_argument(
        "--show-replan",
        choices=("auto", "on", "off"),
        default="on",
        help="Show the contact-triggered stance splice branch; auto follows task.info enable_splice.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    task_info = args.task_info.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    settings = load_settings(task_info)
    show_replan = bool(settings["enable_splice"])
    if args.show_replan != "auto":
        show_replan = args.show_replan == "on"

    configure_style()
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.0), gridspec_kw={"height_ratios": [0.82, 1.25]})
    draw_nominal_panel(axes[0])
    draw_robust_panel(axes[1], settings, show_replan)
    fig.subplots_adjust(left=0.015, right=0.995, top=0.975, bottom=0.025, hspace=0.18)
    draw_robust_expansion_arrows(fig, axes[0], axes[1])
    output_base = output_dir / "paper_robust_phase_explainer"
    save_figure(fig, output_base)

    metadata = {
        "task_info": str(task_info),
        "show_replan": show_replan,
        "task_enable_splice": settings["enable_splice"],
        **settings,
        "implementation_mapping": {
            "window": "PerceptiveLeggedReferenceManager::computeRobustWindows",
            "boundaries": "RobustGuardBoundaryConstraint: g(t_a)=+d, g(t_b)=-d",
            "velocity_envelope": "RobustGuardApproachConstraint and RobustGuardVelocityLowerBoundConstraint",
            "replan": "requestRobustContactSplice/applyPendingSplices when enable_splice=true",
        },
    }
    (output_dir / "robust_phase_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Wrote {output_base.with_suffix('.pdf')}, {output_base.with_suffix('.svg')}, and {output_base.with_suffix('.png')}")


if __name__ == "__main__":
    main()
