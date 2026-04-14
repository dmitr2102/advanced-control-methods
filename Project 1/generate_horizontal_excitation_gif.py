from __future__ import annotations

import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from gif_plot_utils import (
    compute_positive_limits,
    compute_symmetric_limits,
    draw_axes_and_series,
    draw_info_lines,
    standard_layout,
)

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from horizontal_excitation import (  # noqa: E402
    HorizontalExcitationParameters,
    harmonic_horizontal_action,
    horizontal_p,
    side_equilibria,
)


def rk4_step(
    state: tuple[float, float],
    time_value: float,
    dt: float,
    amplitude: float,
    frequency: float,
    params: HorizontalExcitationParameters,
) -> tuple[float, float]:
    action = harmonic_horizontal_action(time_value, amplitude, frequency)

    def add_scaled(base: tuple[float, float], delta: tuple[float, float], scale: float) -> tuple[float, float]:
        return base[0] + scale * delta[0], base[1] + scale * delta[1]

    k1 = horizontal_p(state, action, params)
    k2 = horizontal_p(add_scaled(state, k1, dt / 2.0), action, params)
    k3 = horizontal_p(add_scaled(state, k2, dt / 2.0), action, params)
    k4 = horizontal_p(add_scaled(state, k3, dt), action, params)
    theta = state[0] + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
    theta_dot = state[1] + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
    return theta, theta_dot


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def draw_horizontal_pendulum_panel(
    draw: ImageDraw.ImageDraw,
    panel: tuple[int, int, int, int],
    pivot_base: tuple[int, int],
    rod_length: int,
    theta: float,
    target_theta: float,
    action: float,
    action_scale: float,
) -> None:
    x0, y0, x1, y1 = panel
    pivot_x_base, pivot_y = pivot_base
    draw.rounded_rectangle(panel, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)

    support_offset_x = int((action / max(1.0, action_scale)) * 115.0)
    pivot_x = pivot_x_base + support_offset_x
    bob_x = pivot_x + int(rod_length * math.sin(theta))
    bob_y = pivot_y + int(rod_length * math.cos(theta))

    target_x = pivot_x + int(rod_length * math.sin(target_theta))
    target_y = pivot_y + int(rod_length * math.cos(target_theta))

    draw.line((pivot_x_base - 140, pivot_y, pivot_x_base + 140, pivot_y), fill=(95, 96, 100), width=4)
    draw.line((pivot_x - 90, pivot_y - 18, pivot_x + 90, pivot_y - 18), fill=(140, 140, 145), width=8)
    draw.ellipse((pivot_x - 8, pivot_y - 8, pivot_x + 8, pivot_y + 8), fill=(70, 70, 70))

    draw.line((pivot_x, pivot_y, target_x, target_y), fill=(150, 205, 150), width=2)
    draw.ellipse((target_x - 8, target_y - 8, target_x + 8, target_y + 8), fill=(80, 175, 80))

    draw.line((pivot_x, pivot_y, bob_x, bob_y), fill=(38, 98, 170), width=6)
    draw.ellipse((bob_x - 22, bob_y - 22, bob_x + 22, bob_y + 22), fill=(212, 100, 36), outline=(120, 50, 20))


def draw_frame(
    theta_samples: list[tuple[float, float]],
    error_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    theta: float,
    target_theta: float,
    action: float,
    amplitude: float,
    frequency: float,
    params: HorizontalExcitationParameters,
) -> Image.Image:
    layout = standard_layout()
    width, height = layout["size"]
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.text((40, 20), "Horizontal excitation: stabilization near a side equilibrium", fill=(30, 45, 80), font=font)
    draw_horizontal_pendulum_panel(
        draw,
        layout["left_panel"],
        layout["pivot"],
        int(layout["rod_px"] * params.length),
        theta,
        target_theta,
        action,
        amplitude,
    )

    proximity = 100.0 * abs(target_theta) / (0.5 * math.pi)
    info_lines = [
        "Horizontal harmonic excitation",
        "",
        f"alpha = {amplitude:.1f} m/s^2",
        f"omega = {frequency:.1f} rad/s",
        f"d = {params.damping:.2f}",
        "",
        f"target side equilibrium = {math.degrees(target_theta):.2f} deg",
        f"horizontal proximity = {proximity:.1f}%",
        "",
        "Green marker: averaged side equilibrium",
        "Orange bob: full nonlinear motion",
    ]
    draw_info_lines(draw, layout["info_x"], layout["info_y"], info_lines, layout["info_step"])

    draw_axes_and_series(
        draw,
        layout["plot_top"],
        [{"samples": theta_samples, "color": (38, 98, 170), "label": "theta(t)"}],
        "Absolute angle",
        "t [s]",
        "theta [rad]",
        compute_symmetric_limits((value for _, value in theta_samples), minimum_half_range=1.7),
    )
    draw_axes_and_series(
        draw,
        layout["plot_mid"],
        [{"samples": error_samples, "color": (212, 100, 36), "label": "|theta - theta*|"}],
        "Side-equilibrium tracking error",
        "t [s]",
        "error [rad]",
        compute_positive_limits((value for _, value in error_samples), minimum_upper=0.15),
    )
    draw_axes_and_series(
        draw,
        layout["plot_bot"],
        [{"samples": control_samples, "color": (90, 90, 90), "label": "a(t)"}],
        "Horizontal control acceleration",
        "t [s]",
        "a [m/s^2]",
        compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=amplitude * 1.05),
    )
    return image


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "horizontal_excitation_side_equilibrium.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = HorizontalExcitationParameters(damping=0.5)
    amplitude = 800.0
    frequency = 50.0
    target_pair = side_equilibria(amplitude, frequency, params)
    if target_pair is None:
        raise RuntimeError("Chosen parameters do not produce side equilibria in the averaged model.")
    target_theta = target_pair[0]

    dt = 1.0 / 240.0
    time_value = 0.0
    state = (0.8 * target_theta, 0.0)

    frames: list[Image.Image] = []
    theta_samples: list[tuple[float, float]] = [(0.0, state[0])]
    error_samples: list[tuple[float, float]] = [(0.0, abs(wrap_to_pi(state[0] - target_theta)))]
    control_samples: list[tuple[float, float]] = [(0.0, harmonic_horizontal_action(0.0, amplitude, frequency))]
    window: list[tuple[float, float, float]] = [(0.0, abs(wrap_to_pi(state[0] - target_theta)), abs(state[1]))]
    stable_time = None
    frame_interval = 12

    for step in range(int(12.0 / dt)):
        action = harmonic_horizontal_action(time_value, amplitude, frequency)
        state = rk4_step(state, time_value, dt, amplitude, frequency, params)
        time_value += dt

        tracking_error = abs(wrap_to_pi(state[0] - target_theta))
        theta_samples.append((time_value, state[0]))
        error_samples.append((time_value, tracking_error))
        control_samples.append((time_value, action))
        window.append((time_value, tracking_error, abs(state[1])))
        while window and time_value - window[0][0] > 1.5:
            window.pop(0)

        if step % frame_interval == 0:
            frames.append(
                draw_frame(
                    theta_samples,
                    error_samples,
                    control_samples,
                    state[0],
                    target_theta,
                    action,
                    amplitude,
                    frequency,
                    params,
                )
            )

        if time_value > 4.0 and max(item[1] for item in window) < 0.08 and max(item[2] for item in window) < 0.30:
            stable_time = time_value
            break

    if not frames:
        raise RuntimeError("No frames generated for horizontal excitation GIF.")

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000.0 * dt * frame_interval),
        loop=0,
        disposal=2,
    )
    if stable_time is None:
        print(f"Saved GIF to: {output_path}")
        print(f"Generated {len(frames)} frames over {time_value:.2f} s (no early stopping).")
    else:
        print(f"Saved GIF to: {output_path}")
        print(f"Generated {len(frames)} frames up to t = {stable_time:.2f} s")


if __name__ == "__main__":
    main()
