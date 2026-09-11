#!/usr/bin/env python3
"""Shared plotting style adapted from ``~/paper_quality_plot.matlab``."""

import matplotlib.pyplot as plt
from pathlib import Path
from matplotlib.legend import Legend
from matplotlib.text import Text


# Okabe-Ito orange/blue pair (candidate B).
NOMINAL_COLOR = (0.9020, 0.6235, 0.0)     # #E69F00
PROPOSED_COLOR = (0.0, 0.4471, 0.6980)    # #0072B2
FAILURE_COLOR = (0.9153, 0.2816, 0.2878)
PERCEIVED_COLOR = (0.0, 0.6196, 0.4510)    # #009E73
REFERENCE_COLOR = (0.24, 0.24, 0.24)
GRID_COLOR = (0.72, 0.72, 0.72)
COMPONENT_COLORS = (PROPOSED_COLOR, NOMINAL_COLOR, PERCEIVED_COLOR)
DESCENT_START_COLOR = (0.5804, 0.4039, 0.7412)
DESCENT_COMPLETE_COLOR = (0.0, 0.6196, 0.4510)

NOMINAL_MARKER = "o"
PROPOSED_MARKER = "s"
LINE_WIDTH = 2.25
MARKER_SIZE = 8.0
TITLE_SIZE = 19
AXIS_LABEL_SIZE = 17
TICK_LABEL_SIZE = 13
LEGEND_SIZE = 15
PANEL_WIDTH = 5.0
PANEL_HEIGHT = 4.0
SAVE_DPI = 300


def paper_figsize(columns=1, rows=1, *, width_scale=1.0, height_scale=1.0):
    """Return the 5:4-per-panel geometry used by the MATLAB template."""
    return (
        PANEL_WIDTH * columns * width_scale,
        PANEL_HEIGHT * rows * height_scale,
    )


def apply_paper_style():
    """Configure Matplotlib to match the MATLAB repository's paper figures."""
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["STIXGeneral", "Times New Roman", "Liberation Serif"],
            "mathtext.fontset": "stix",
            "axes.titlesize": TITLE_SIZE,
            "axes.titleweight": "normal",
            "axes.labelsize": AXIS_LABEL_SIZE,
            "axes.linewidth": 0.8,
            "axes.edgecolor": "0.25",
            "xtick.labelsize": TICK_LABEL_SIZE,
            "ytick.labelsize": TICK_LABEL_SIZE,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.top": True,
            "ytick.right": True,
            "legend.fontsize": LEGEND_SIZE,
            "legend.frameon": True,
            "legend.fancybox": False,
            "legend.framealpha": 1.0,
            "lines.linewidth": LINE_WIDTH,
            "lines.markersize": MARKER_SIZE,
            "figure.titlesize": TITLE_SIZE,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.dpi": SAVE_DPI,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def apply_figure_font_sizes(fig, output_path):
    """Match Fig. 1(b) only for result-gallery plots, before layout."""
    results_root = Path(__file__).resolve().parent / "results"
    try:
        relative = Path(output_path).resolve().relative_to(results_root.resolve())
    except ValueError:
        return
    if not {"all_visualization", "all_visualizations"}.intersection(relative.parts[:-1]):
        return
    for text in fig.findobj(match=Text):
        text.set_fontsize(TICK_LABEL_SIZE)
    for axis in fig.axes:
        for coordinate in (axis.xaxis, axis.yaxis):
            coordinate.label.set_fontsize(AXIS_LABEL_SIZE)
            coordinate.set_tick_params(which="both", labelsize=TICK_LABEL_SIZE)
            coordinate.get_offset_text().set_fontsize(TICK_LABEL_SIZE)
        for title in (axis.title, axis._left_title, axis._right_title):
            title.set_fontsize(TITLE_SIZE)
        for table in axis.tables:
            table.auto_set_font_size(False)
            table.set_fontsize(TICK_LABEL_SIZE)
    for legend in fig.findobj(match=Legend):
        for text in legend.get_texts():
            text.set_fontsize(LEGEND_SIZE)
        legend.get_title().set_fontsize(LEGEND_SIZE)
    for name, size in (("_suptitle", TITLE_SIZE),
                       ("_supxlabel", AXIS_LABEL_SIZE),
                       ("_supylabel", AXIS_LABEL_SIZE)):
        text = getattr(fig, name, None)
        if text is not None:
            text.set_fontsize(size)


def style_paper_axis(axis, *, minor=False, grid_axis="both"):
    """Apply the MATLAB-style frame, ticks, and dotted grid to one axis."""
    axis.set_axisbelow(True)
    axis.grid(
        True, which="major", axis=grid_axis,
        color=GRID_COLOR, linestyle=":", linewidth=0.75,
    )
    if minor:
        axis.minorticks_on()
        axis.grid(
            True, which="minor", axis=grid_axis,
            color=GRID_COLOR, linestyle=":", linewidth=0.45, alpha=0.55,
        )
    for spine in axis.spines.values():
        spine.set_visible(True)
        spine.set_color("0.25")
        spine.set_linewidth(0.8)
    axis.tick_params(which="major", direction="in", top=True, right=True, length=6, width=0.8)
    axis.tick_params(which="minor", direction="in", top=True, right=True, length=3, width=0.6)


def paper_legend(owner, *args, **kwargs):
    """Create the square, white legend boxes used by the MATLAB examples."""
    kwargs.setdefault("frameon", True)
    kwargs.setdefault("fancybox", False)
    kwargs.setdefault("framealpha", 1.0)
    kwargs.setdefault("edgecolor", "0.35")
    kwargs.setdefault("facecolor", "white")
    kwargs.setdefault("fontsize", LEGEND_SIZE)
    return owner.legend(*args, **kwargs)
