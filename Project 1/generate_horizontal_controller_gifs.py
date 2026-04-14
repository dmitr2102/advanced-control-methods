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

from horizontal_controllers import (  # noqa: E402
    HorizontalAveragedEnergyController,
    HorizontalCycleEnergyPIDController,
    HorizontalHarmonicController,
    HorizontalLyapunovController,
    wrap_to_pi,
)
from horizontal_excitation import HorizontalExcitationParameters  # noqa: E402
from horizontal_simulation import HorizontalPendulumSimulator, HorizontalSimulationConfig  # noqa: E402


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

    draw.line((pivot_x_base - 150, pivot_y, pivot_x_base + 150, pivot_y), fill=(95, 96, 100), width=4)
    draw.line((pivot_x - 90, pivot_y - 18, pivot_x + 90, pivot_y - 18), fill=(140, 140, 145), width=8)
    draw.ellipse((pivot_x - 8, pivot_y - 8, pivot_x + 8, pivot_y + 8), fill=(70, 70, 70))
    draw.line((pivot_x, pivot_y, target_x, target_y), fill=(150, 205, 150), width=2)
    draw.ellipse((target_x - 8, target_y - 8, target_x + 8, target_y + 8), fill=(80, 175, 80))
    draw.line((pivot_x, pivot_y, bob_x, bob_y), fill=(38, 98, 170), width=6)
    draw.ellipse((bob_x - 22, bob_y - 22, bob_x + 22, bob_y + 22), fill=(212, 100, 36), outline=(120, 50, 20))


def build_info_lines(controller_name: str, controller: object, params: HorizontalExcitationParameters, target_theta: float) -> list[str]:
    target_deg = math.degrees(target_theta)
    proximity = 100.0 * abs(target_theta) / (0.5 * math.pi)
    if controller_name == "horizontal_harmonic":
        return [
            "Horizontal harmonic controller",
            "",
            f"alpha = {controller.amplitude:.1f} m/s^2",
            f"omega = {controller.frequency:.1f} rad/s",
            f"d = {params.damping:.2f}",
            "",
            f"target side equilibrium = {target_deg:.2f} deg",
            f"horizontal proximity = {proximity:.1f}%",
            "",
            "Green marker: target side equilibrium",
            "Orange bob: current nonlinear motion",
        ]
    if controller_name == "horizontal_lyapunov":
        return [
            "Horizontal Lyapunov controller",
            "",
            f"omega = {controller.carrier_frequency:.1f} rad/s",
            f"k_e = {controller.cosine_error_gain:.4f}",
            f"k_d = {controller.rate_gain:.4f}",
            f"d = {params.damping:.2f}",
            "",
            f"target side equilibrium = {target_deg:.2f} deg",
            f"horizontal proximity = {proximity:.1f}%",
            "",
            "Amplitude is shaped from",
            "the averaged side-equilibrium cosine.",
        ]
    if controller_name == "horizontal_averaged_energy":
        return [
            "Horizontal averaged-energy",
            "",
            f"omega = {controller.carrier_frequency:.1f} rad/s",
            f"k_E = {controller.angle_energy_gain:.2f}",
            f"k_rate = {controller.rate_energy_gain:.2f}",
            f"d = {params.damping:.2f}",
            "",
            f"target side equilibrium = {target_deg:.2f} deg",
            f"horizontal proximity = {proximity:.1f}%",
            "",
            "Amplitude grows with",
            "the averaged side-equilibrium energy.",
        ]
    return [
        "Horizontal PID cycle-energy",
        "",
        f"omega = {controller.carrier_frequency:.1f} rad/s",
        f"kp = {controller.kp:.3f}",
        f"ki = {controller.ki:.3f}",
        f"kd = {controller.kd:.3f}",
        f"d = {params.damping:.2f}",
        "",
        f"target side equilibrium = {target_deg:.2f} deg",
        f"horizontal proximity = {proximity:.1f}%",
        "",
        "Amplitude is updated from",
        "cycle-averaged side-equilibrium energy.",
    ]


def draw_frame(
    title: str,
    controller_name: str,
    controller: object,
    params: HorizontalExcitationParameters,
    theta_samples: list[tuple[float, float]],
    error_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    theta: float,
    target_theta: float,
    action: float,
    action_scale: float,
) -> Image.Image:
    layout = standard_layout()
    width, height = layout["size"]
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.text((40, 20), title, fill=(30, 45, 80), font=font)
    draw_horizontal_pendulum_panel(
        draw,
        layout["left_panel"],
        layout["pivot"],
        int(layout["rod_px"] * params.length),
        theta,
        target_theta,
        action,
        action_scale,
    )
    draw_info_lines(draw, layout["info_x"], layout["info_y"], build_info_lines(controller_name, controller, params, target_theta), layout["info_step"])

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
        compute_positive_limits((value for _, value in error_samples), minimum_upper=0.12),
    )
    draw_axes_and_series(
        draw,
        layout["plot_bot"],
        [{"samples": control_samples, "color": (90, 90, 90), "label": "a(t)"}],
        "Horizontal control acceleration",
        "t [s]",
        "a [m/s^2]",
        compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=action_scale * 0.7),
    )
    return image


def generate_gif(
    controller: object,
    title: str,
    output_name: str,
    params: HorizontalExcitationParameters,
    horizon: float = 12.0,
    tube_radius: float = 0.08,
) -> None:
    target_theta = controller.target_angle
    simulator = HorizontalPendulumSimulator(
        params,
        HorizontalSimulationConfig(
            dt=1.0 / 240.0,
            initial_theta=0.8 * target_theta,
            initial_theta_dot=0.0,
            max_abs_action=1200.0,
        ),
    )
    if hasattr(controller, "reset"):
        controller.reset()

    output_path = PROJECT_ROOT / "animations" / output_name
    output_path.parent.mkdir(exist_ok=True)

    controller_name = controller.name
    action_scale = getattr(controller, "reference_amplitude", getattr(controller, "amplitude", 800.0))
    frames: list[Image.Image] = []
    theta_samples = [(0.0, simulator.state.theta)]
    error_samples = [(0.0, abs(wrap_to_pi(simulator.state.theta - target_theta)))]
    control_samples = [(0.0, 0.0)]
    stable_time = None
    frame_interval = 12

    for step in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        theta_error = abs(wrap_to_pi(state.theta - target_theta))
        theta_samples.append((state.time_value, state.theta))
        error_samples.append((state.time_value, theta_error))
        control_samples.append((state.time_value, state.action))

        if step % frame_interval == 0:
            frames.append(
                draw_frame(
                    title,
                    controller_name,
                    controller,
                    params,
                    theta_samples,
                    error_samples,
                    control_samples,
                    state.theta,
                    target_theta,
                    state.action,
                    action_scale,
                )
            )

        if stable_time is None and state.time_value > 1.0 and all(value <= tube_radius for _, value in error_samples[-360:]):
            stable_time = state.time_value

    if not frames:
        raise RuntimeError(f"No frames generated for {output_name}.")

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000.0 * simulator.config.dt * frame_interval),
        loop=0,
        disposal=2,
    )
    if stable_time is None:
        print(f"Saved GIF to: {output_path} (no early convergence marker)")
    else:
        print(f"Saved GIF to: {output_path} (entered tube near t = {stable_time:.2f} s)")


def main() -> None:
    params = HorizontalExcitationParameters(damping=0.5)
    generate_gif(
        HorizontalHarmonicController(gravity=params.gravity, length=params.length),
        "Horizontal harmonic stabilization near a side equilibrium",
        "horizontal_harmonic_stabilization.gif",
        params,
        horizon=12.0,
    )
    generate_gif(
        HorizontalLyapunovController(gravity=params.gravity, length=params.length),
        "Horizontal Lyapunov stabilization near a side equilibrium",
        "horizontal_lyapunov_stabilization.gif",
        params,
        horizon=8.0,
    )
    generate_gif(
        HorizontalAveragedEnergyController(gravity=params.gravity, length=params.length),
        "Horizontal averaged-energy stabilization near a side equilibrium",
        "horizontal_averaged_energy_stabilization.gif",
        params,
        horizon=10.0,
    )
    generate_gif(
        HorizontalCycleEnergyPIDController(gravity=params.gravity, length=params.length),
        "Horizontal PID cycle-energy stabilization near a side equilibrium",
        "horizontal_pid_cycle_energy_stabilization.gif",
        params,
        horizon=12.0,
    )


if __name__ == "__main__":
    main()
