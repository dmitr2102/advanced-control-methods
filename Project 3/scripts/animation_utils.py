from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def animate_pulleys(
    result: dict[str, np.ndarray],
    output_path: Path,
    fps: int = 12,
    title: str | None = None,
    annotation_lines: list[str] | None = None,
) -> None:
    time = result["time"]
    state = result["state"]
    reference = result["reference"]
    deformation = result["cable_deformation"]
    tension_1 = result["tension_1"]
    tension_2 = result["tension_2"]
    min_tension = float(min(np.min(tension_1), np.min(tension_2)))
    max_tension = float(max(np.max(tension_1), np.max(tension_2)))
    frame_count = int(np.ceil(time[-1] * fps)) + 1
    frame_indices = np.linspace(0, len(time) - 1, frame_count).astype(int)

    width, height = 720, 405
    motor_center = (220, 205)
    target_center = (500, 205)
    radius = 58
    rod_length_px = 125
    frames: list[Image.Image] = []
    animation_title = title or "Cable-pulley simulation"
    extra_annotations = annotation_lines or []

    def angle_endpoint(center: tuple[int, int], angle: float, length: int = radius) -> tuple[int, int]:
        return (
            int(center[0] + length * np.sin(angle)),
            int(center[1] + length * np.cos(angle)),
        )

    def dashed_line(
        draw: ImageDraw.ImageDraw,
        start: tuple[int, int],
        end: tuple[int, int],
        fill: tuple[int, int, int],
        width: int,
        dash_length: int = 9,
        gap_length: int = 6,
    ) -> None:
        start_arr = np.array(start, dtype=float)
        end_arr = np.array(end, dtype=float)
        vector = end_arr - start_arr
        length = float(np.linalg.norm(vector))
        if length <= 1.0e-12:
            return
        direction = vector / length
        cursor = 0.0
        while cursor < length:
            dash_end = min(cursor + dash_length, length)
            p0 = start_arr + direction * cursor
            p1 = start_arr + direction * dash_end
            draw.line([tuple(p0.astype(int)), tuple(p1.astype(int))], fill=fill, width=width)
            cursor += dash_length + gap_length

    def tension_color(tension_value: float) -> tuple[int, int, int]:
        tension_range = max_tension - min_tension
        if tension_range <= 1.0e-12:
            return (0, 160, 0)
        level = min(max((tension_value - min_tension) / tension_range, 0.0), 1.0)
        if level <= 0.5:
            local = level / 0.5
            red = int(255 * local)
            green = int(160 + (220 - 160) * local)
            return (red, green, 0)
        local = (level - 0.5) / 0.5
        red = 255
        green = int(220 * (1.0 - local))
        return (red, green, 0)

    def draw_centered_text(
        draw: ImageDraw.ImageDraw,
        center_x: int,
        y: int,
        text: str,
        fill: tuple[int, int, int],
    ) -> None:
        bbox = draw.textbbox((0, 0), text)
        text_width = bbox[2] - bbox[0]
        draw.text((center_x - text_width // 2, y), text, fill=fill)

    for idx in frame_indices:
        theta_l = float(state[idx, 0])
        theta_m = float(state[idx, 2])
        ref = float(reference[idx])
        voltage = float(result["voltage"][idx])
        current_deformation = float(deformation[idx])
        top_tension = float(tension_1[idx])
        bottom_tension = float(tension_2[idx])
        top_cable_color = tension_color(top_tension)
        bottom_cable_color = tension_color(bottom_tension)

        frame = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(frame)

        draw_centered_text(draw, width // 2, 10, animation_title, fill=(0, 0, 0))
        draw_centered_text(draw, motor_center[0], 116, "motor pulley", fill=(31, 119, 180))
        draw_centered_text(draw, target_center[0], 116, "target pulley", fill=(255, 127, 14))
        draw_centered_text(
            draw,
            target_center[0],
            132,
            "blue dashed: motor pulley",
            fill=(31, 119, 180),
        )

        draw.line(
            [(motor_center[0], motor_center[1] - radius), (target_center[0], target_center[1] - radius)],
            fill=top_cable_color,
            width=5,
        )
        draw.line(
            [(motor_center[0], motor_center[1] + radius), (target_center[0], target_center[1] + radius)],
            fill=bottom_cable_color,
            width=5,
        )
        draw.ellipse(
            [
                motor_center[0] - radius,
                motor_center[1] - radius,
                motor_center[0] + radius,
                motor_center[1] + radius,
            ],
            outline=(31, 119, 180),
            width=4,
        )
        draw.ellipse(
            [
                target_center[0] - radius,
                target_center[1] - radius,
                target_center[0] + radius,
                target_center[1] + radius,
            ],
            outline=(255, 127, 14),
            width=4,
        )
        draw.line([motor_center, angle_endpoint(motor_center, theta_m)], fill=(31, 119, 180), width=5)
        draw.line([target_center, angle_endpoint(target_center, theta_l)], fill=(255, 127, 14), width=5)
        rod_end = angle_endpoint(target_center, theta_l, rod_length_px)
        draw.line([target_center, rod_end], fill=(120, 120, 120), width=8)
        draw.ellipse([rod_end[0] - 8, rod_end[1] - 8, rod_end[0] + 8, rod_end[1] + 8], fill=(90, 90, 90))
        dashed_line(
            draw,
            target_center,
            angle_endpoint(target_center, theta_m, radius + 18),
            fill=(31, 119, 180),
            width=3,
        )
        ref_end = angle_endpoint(target_center, ref)
        draw.line([target_center, ref_end], fill=(0, 0, 0), width=3)
        draw.ellipse([ref_end[0] - 4, ref_end[1] - 4, ref_end[0] + 4, ref_end[1] + 4], fill=(0, 0, 0))

        draw.text((28, 34), f"t = {time[idx]:.2f} s", fill=(0, 0, 0))
        draw.text((28, 56), f"theta_ref = {ref:+.2f} rad", fill=(0, 0, 0))
        draw.text((28, 78), f"theta_l = {theta_l:+.2f} rad", fill=(0, 0, 0))
        draw.text((28, 100), f"V = {voltage:+.1f} V", fill=(0, 0, 0))
        draw.text((28, 122), f"q = {current_deformation:+.3f} m", fill=(0, 0, 0))
        draw.text((28, 144), f"T_top = {top_tension:.1f} N", fill=top_cable_color)
        draw.text((28, 166), f"T_bottom = {bottom_tension:.1f} N", fill=bottom_cable_color)
        draw.text((28, 188), f"rod load torque = {result['rod_gravity_torque'][idx]:+.2f} Nm", fill=(0, 0, 0))
        for line_idx, annotation in enumerate(extra_annotations):
            draw.text((28, 210 + 22 * line_idx), annotation, fill=(0, 0, 0))
        draw.rectangle([28, 365, 178, 378], fill=(0, 160, 0))
        draw.rectangle([178, 365, 328, 378], fill=(255, 220, 0))
        draw.rectangle([328, 365, 478, 378], fill=(255, 0, 0))
        draw.text((28, 382), f"low T ({min_tension:.0f} N)", fill=(0, 0, 0))
        draw.text((215, 382), "mid T", fill=(0, 0, 0))
        draw.text((390, 382), f"high T ({max_tension:.0f} N)", fill=(0, 0, 0))
        frames.append(frame)

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / fps),
        loop=0,
        optimize=True,
    )
