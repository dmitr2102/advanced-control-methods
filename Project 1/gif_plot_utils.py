from __future__ import annotations

import math
from typing import Iterable

from PIL import ImageDraw, ImageFont


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


def draw_axes_and_series(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    series_list: list[dict],
    title: str,
    x_label: str,
    y_label: str,
    y_limits: tuple[float, float],
    x_ticks: int = 4,
    y_ticks: int = 4,
) -> None:
    x0, y0, x1, y1 = rect
    font = ImageFont.load_default()
    draw.rounded_rectangle(rect, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)
    draw.text((x0 + 18, y0 + 12), title, fill=(30, 45, 80), font=font)

    left = x0 + 64
    top = y0 + 44
    right = x1 - 18
    bottom = y1 - 42
    draw.line((left, bottom, right, bottom), fill=(90, 90, 90), width=2)
    draw.line((left, top, left, bottom), fill=(90, 90, 90), width=2)

    all_samples = [sample for series in series_list for sample in series["samples"]]
    if not all_samples:
        return

    t_max = max(sample[0] for sample in all_samples)
    t_max = max(t_max, 1.0)
    y_min, y_max = y_limits
    if y_max - y_min < 1e-9:
        y_max = y_min + 1.0

    for idx in range(y_ticks + 1):
        value = y_min + idx * (y_max - y_min) / y_ticks
        y = bottom - ((value - y_min) / (y_max - y_min)) * (bottom - top)
        draw.line((left, y, right, y), fill=(225, 225, 225), width=1)
        draw.text((x0 + 8, y - 7), f"{value:.2f}", fill=(90, 90, 90), font=font)

    for idx in range(x_ticks + 1):
        value = idx * t_max / x_ticks
        x = left + (value / t_max) * (right - left)
        draw.line((x, top, x, bottom), fill=(230, 230, 230), width=1)
        draw.text((x - 10, bottom + 8), f"{value:.1f}", fill=(90, 90, 90), font=font)

    if y_min < 0.0 < y_max:
        zero_y = bottom - ((0.0 - y_min) / (y_max - y_min)) * (bottom - top)
        draw.line((left, zero_y, right, zero_y), fill=(160, 160, 160), width=1)

    for series in series_list:
        points = []
        for time_value, value in series["samples"]:
            x = left + (time_value / t_max) * (right - left)
            clipped = min(y_max, max(y_min, value))
            y = bottom - ((clipped - y_min) / (y_max - y_min)) * (bottom - top)
            points.append((x, y))
        if len(points) > 1:
            draw.line(points, fill=series["color"], width=3)

    legend_x = right - 160
    legend_y = y0 + 14
    for idx, series in enumerate(series_list):
        y = legend_y + 18 * idx
        draw.line((legend_x, y + 6, legend_x + 22, y + 6), fill=series["color"], width=3)
        draw.text((legend_x + 28, y), series["label"], fill=(55, 55, 60), font=font)

    draw.text((right - 40, bottom + 24), x_label, fill=(55, 55, 60), font=font)
    draw.text((x0 + 10, top - 18), y_label, fill=(55, 55, 60), font=font)


def standard_layout() -> dict:
    return {
        "size": (1400, 920),
        "left_panel": (40, 60, 520, 880),
        "plot_top": (560, 60, 1360, 310),
        "plot_mid": (560, 340, 1360, 590),
        "plot_bot": (560, 620, 1360, 870),
        "pivot": (280, 380),
        "rod_px": 215,
        "info_x": 78,
        "info_y": 690,
        "info_step": 24,
    }


def draw_pendulum_panel(
    draw: ImageDraw.ImageDraw,
    panel: tuple[int, int, int, int],
    pivot: tuple[int, int],
    rod_length: int,
    phi: float,
    action: float,
    action_scale: float = 140.0,
) -> None:
    x0, y0, x1, y1 = panel
    pivot_x, pivot_y = pivot
    draw.rounded_rectangle(panel, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)
    support_offset = int((action / max(1.0, action_scale)) * 55.0)
    support_y = pivot_y + support_offset
    bob_x = pivot_x - int(rod_length * math.sin(phi))
    bob_y = support_y - int(rod_length * math.cos(phi))
    draw.line((pivot_x - 120, support_y, pivot_x + 120, support_y), fill=(95, 96, 100), width=8)
    draw.line((pivot_x, support_y, pivot_x, support_y - rod_length), fill=(182, 212, 182), width=2)
    draw.ellipse((pivot_x - 8, support_y - 8, pivot_x + 8, support_y + 8), fill=(70, 70, 70))
    draw.line((pivot_x, support_y, bob_x, bob_y), fill=(38, 98, 170), width=6)
    draw.ellipse((bob_x - 22, bob_y - 22, bob_x + 22, bob_y + 22), fill=(212, 100, 36), outline=(120, 50, 20))
    draw.ellipse((pivot_x - 6, support_y - rod_length - 6, pivot_x + 6, support_y - rod_length + 6), fill=(80, 175, 80))


def draw_info_lines(
    draw: ImageDraw.ImageDraw,
    start_x: int,
    start_y: int,
    lines: list[str],
    step: int = 24,
) -> None:
    font = ImageFont.load_default()
    y = start_y
    for line in lines:
        draw.text((start_x, y), line, fill=(40, 40, 45), font=font)
        y += step
