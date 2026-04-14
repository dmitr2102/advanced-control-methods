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
    draw_pendulum_panel,
    standard_layout,
)

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.averaged_energy import AveragedEnergyController
from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters


def averaged_energy_value(phi: float, phi_dot: float, amplitude: float, omega: float, params: PendulumParameters) -> float:
    kinetic = 0.5 * phi_dot * phi_dot
    shaped = (amplitude * amplitude) / (4.0 * params.length * params.length * omega * omega)
    shaped *= math.sin(phi) ** 2
    potential = (params.gravity / params.length) * (math.cos(phi) - 1.0)
    return kinetic + shaped + potential


def draw_frame(
    phi_samples: list[tuple[float, float]],
    v_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_time: float,
    current_phi: float,
    current_action: float,
    current_amplitude: float,
    params: PendulumParameters,
) -> Image.Image:
    layout = standard_layout()
    width, height = layout["size"]
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.text((40, 20), "Averaged-energy controller for the Kapitza pendulum", fill=(30, 45, 80), font=font)

    draw_pendulum_panel(draw, layout["left_panel"], layout["pivot"], int(layout["rod_px"] * params.length), current_phi, current_action)

    info_lines = [
        f"d = {params.damping: .2f}",
        "",
        "Green marker: upright target",
        "Orange bob: current pendulum state",
        "",
        "Control law:",
        "a(t) = alpha(s) cos(omega t)",
        "with amplitude shaping based on",
        "the averaged Kapitza energy.",
    ]
    draw_info_lines(draw, layout["info_x"], layout["info_y"], info_lines, layout["info_step"])

    draw_axes_and_series(
        draw,
        layout["plot_top"],
        [{"samples": phi_samples, "color": (38, 98, 170), "label": "phi(t)"}],
        "Angle deviation",
        "t [s]",
        "phi [rad]",
        compute_symmetric_limits((value for _, value in phi_samples), minimum_half_range=0.15),
    )
    value_limits = compute_positive_limits((value for _, value in v_samples), minimum_upper=0.25)
    control_limits = compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=60.0)
    draw_axes_and_series(
        draw,
        layout["plot_mid"],
        [{"samples": v_samples, "color": (212, 100, 36), "label": "V(t)"}],
        "Averaged-energy Lyapunov value",
        "t [s]",
        "V",
        value_limits,
    )
    draw_axes_and_series(
        draw,
        layout["plot_bot"],
        [{"samples": control_samples, "color": (90, 90, 90), "label": "a(t)"}],
        "Control signal",
        "t [s]",
        "a [m/s^2]",
        control_limits,
    )

    return image


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "averaged_energy_stabilization_tuned.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = PendulumParameters(damping=0.25)
    plant = KapitzaPendulumPlant(params)
    controller = AveragedEnergyController(
        gravity=params.gravity,
        length=params.length,
        carrier_frequency=24.0,
        target_stiffness=10.4,
        angle_gain=6.0,
        rate_gain=0.8,
        min_amplitude=63.0,
        max_amplitude=111.0,
    )
    simulator = PendulumSimulator(
        plant=plant,
        config=SimulationConfig(
            dt=1.0 / 240.0,
            initial_phi=0.2,
            initial_phi_dot=0.0,
            max_abs_action=140.0,
        ),
    )

    frames: list[Image.Image] = []
    phi_samples: list[tuple[float, float]] = []
    v_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    window: list[tuple[float, float, float]] = []
    stable_time = None
    frame_interval = 12
    max_steps = int(24.0 / simulator.config.dt)

    for step in range(max_steps):
        state = simulator.step(controller)
        carrier = math.cos(controller.phase)
        amplitude = abs(state.action / carrier) if abs(carrier) > 1e-6 else controller.max_amplitude
        value = averaged_energy_value(state.phi, state.phi_dot, amplitude, controller.carrier_frequency, params)

        phi_samples.append((state.time_value, state.phi))
        v_samples.append((state.time_value, value))
        control_samples.append((state.time_value, state.action))
        window.append((state.time_value, abs(state.phi), abs(state.phi_dot)))
        while window and state.time_value - window[0][0] > 1.5:
            window.pop(0)

        if step % frame_interval == 0:
            frames.append(
                draw_frame(
                    phi_samples=phi_samples,
                    v_samples=v_samples,
                    control_samples=control_samples,
                    current_time=state.time_value,
                    current_phi=state.phi,
                    current_action=state.action,
                    current_amplitude=amplitude,
                    params=params,
                )
            )

        if state.time_value > 6.0 and max(v[1] for v in window) < 0.10 and max(v[2] for v in window) < 0.40:
            stable_time = state.time_value
            break

    if stable_time is None:
        raise RuntimeError("Averaged-energy controller did not stabilize the pendulum within the simulation horizon.")

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000.0 * simulator.config.dt * frame_interval),
        loop=0,
        disposal=2,
    )

    print(f"Saved GIF to: {output_path}")
    print(f"Generated {len(frames)} frames up to t = {stable_time:.2f} s")


if __name__ == "__main__":
    main()
