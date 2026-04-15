from __future__ import annotations

import io
import math
from typing import Iterable

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib import patches
from PIL import Image


BG = "#f5f4f0"
PANEL = "#fcfaf5"
BLUE = "#2662AA"
ORANGE = "#D46424"
GREEN = "#50AF50"
DARK = "#404045"
GRAY = "#606064"


def compute_symmetric_limits(values: Iterable[float], minimum_half_range: float, margin_ratio: float = 0.12) -> tuple[float, float]:
    values = list(values)
    half_range = max(minimum_half_range, max((abs(v) for v in values), default=0.0))
    half_range *= 1.0 + margin_ratio
    return -half_range, half_range


def compute_positive_limits(values: Iterable[float], minimum_upper: float, margin_ratio: float = 0.12) -> tuple[float, float]:
    values = list(values)
    lower = min(0.0, min(values, default=0.0))
    upper = max(minimum_upper, max(values, default=minimum_upper))
    span = upper - lower
    pad = max(1e-9, span * margin_ratio)
    return lower - 0.15 * pad, upper + pad


def _styled_axis(ax, title: str, xlabel: str, ylabel: str, y_limits: tuple[float, float]) -> None:
    ax.set_facecolor(PANEL)
    ax.set_title(title, fontsize=18, color="#1e2d50", pad=10)
    ax.set_xlabel(xlabel, fontsize=15)
    ax.set_ylabel(ylabel, fontsize=15)
    ax.set_ylim(*y_limits)
    ax.grid(True, alpha=0.28)
    ax.tick_params(labelsize=12)
    for spine in ax.spines.values():
        spine.set_color("#cccccc")


def _draw_pendulum(
    ax,
    phi: float,
    rod_length: float,
    show_target: bool = True,
    y_limits: tuple[float, float] | None = None,
) -> None:
    ax.set_facecolor(PANEL)
    ax.set_aspect("equal")
    ax.set_xlim(-1.6 * rod_length, 1.6 * rod_length)
    if y_limits is None:
        y_limits = (-2.05 * rod_length, 1.55 * rod_length)
    ax.set_ylim(*y_limits)
    ax.axis("off")

    pivot_x = 0.0
    pivot_y = 0.0
    bob_x = rod_length * math.sin(phi)
    bob_y = rod_length * math.cos(phi)

    ax.plot([-0.9 * rod_length, 0.9 * rod_length], [pivot_y, pivot_y], color="#5f6064", linewidth=6)
    ax.plot([pivot_x, pivot_x], [pivot_y, pivot_y + rod_length], color="#b6d4b6", linewidth=2)
    ax.add_patch(patches.Circle((pivot_x, pivot_y), 0.05 * rod_length, color="#444444"))
    if show_target:
        ax.add_patch(patches.Circle((pivot_x, pivot_y + rod_length), 0.04 * rod_length, color=GREEN, alpha=0.9))
    ax.plot([pivot_x, bob_x], [pivot_y, bob_y], color=BLUE, linewidth=5)
    ax.add_patch(patches.Circle((bob_x, bob_y), 0.11 * rod_length, facecolor=ORANGE, edgecolor="#783214", linewidth=1.5))


def render_standard_frame(
    title: str,
    current_phi: float,
    rod_length: float,
    info_lines: list[str],
    plot_defs: list[dict],
    *,
    title_fontsize: int = 26,
    title_y: float = 0.972,
    top_margin: float = 0.885,
    info_x: float = 0.05,
    info_y: float = 0.02,
    info_fontsize: int = 14,
    pendulum_y_limits: tuple[float, float] | None = None,
) -> Image.Image:
    fig = plt.figure(figsize=(16, 10), dpi=140, facecolor=BG)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.65], height_ratios=[1, 1, 1])
    fig.subplots_adjust(left=0.045, right=0.985, bottom=0.055, top=top_margin, wspace=0.15, hspace=0.34)

    fig.suptitle(title, fontsize=title_fontsize, fontweight="bold", color="#1e2d50", y=title_y)

    ax_left = fig.add_subplot(gs[:, 0])
    _draw_pendulum(ax_left, current_phi, rod_length, y_limits=pendulum_y_limits)
    info_text = "\n".join(info_lines)
    ax_left.text(
        info_x,
        info_y,
        info_text,
        transform=ax_left.transAxes,
        fontsize=info_fontsize,
        va="bottom",
        ha="left",
        color=DARK,
        linespacing=1.15,
    )

    axes = [
        fig.add_subplot(gs[0, 1]),
        fig.add_subplot(gs[1, 1]),
        fig.add_subplot(gs[2, 1]),
    ]

    for ax, plot_def in zip(axes, plot_defs):
        _styled_axis(ax, plot_def["title"], plot_def["x_label"], plot_def["y_label"], plot_def["y_limits"])
        for series in plot_def["series"]:
            if not series["samples"]:
                continue
            xs = [x for x, _ in series["samples"]]
            ys = [y for _, y in series["samples"]]
            ax.plot(xs, ys, color=series["color"], linewidth=2.0, label=series["label"])
        if any(series["samples"] for series in plot_def["series"]):
            ax.legend(loc="upper right", fontsize=11, frameon=True)

    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", facecolor=fig.get_facecolor())
    plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert("P", palette=Image.ADAPTIVE)
