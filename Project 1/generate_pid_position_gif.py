from __future__ import annotations

import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from gif_plot_utils import compute_symmetric_limits, draw_axes_and_series, draw_info_lines, draw_pendulum_panel, standard_layout

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.pid import PositionPIDController
from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters


def draw_frame(
    phi_samples: list[tuple[float, float]],
    amp_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_time: float,
    current_phi: float,
    current_action: float,
    current_amplitude: float,
    params: PendulumParameters,
    controller: PositionPIDController,
) -> Image.Image:
    layout = standard_layout()
    width, height = layout["size"]
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.text((40, 20), "PID position controller for the Kapitza pendulum", fill=(30, 45, 80), font=font)

    draw_pendulum_panel(draw, layout["left_panel"], layout["pivot"], int(layout["rod_px"] * params.length), current_phi, current_action)

    lines = [
        "",
        "Law:",
        "alpha = base + PID(|phi|)",
        "a(t) = alpha cos(omega t)",
        "",
        f"kp = {controller.kp:.1f}",
        f"ki = {controller.ki:.1f}",
        f"kd = {controller.kd:.1f}",
    ]
    draw_info_lines(draw, layout["info_x"], layout["info_y"], lines, layout["info_step"])

    draw_axes_and_series(
        draw,
        layout["plot_top"],
        [{"samples": phi_samples, "color": (38, 98, 170), "label": "phi(t)"}],
        "Angle deviation",
        "t [s]",
        "phi [rad]",
        compute_symmetric_limits((value for _, value in phi_samples), minimum_half_range=0.15),
    )
    draw_axes_and_series(
        draw,
        layout["plot_mid"],
        [{"samples": amp_samples, "color": (212, 100, 36), "label": "alpha(t)"}],
        "Carrier amplitude",
        "t [s]",
        "alpha [m/s^2]",
        (60.0, 140.0),
    )
    draw_axes_and_series(
        draw,
        layout["plot_bot"],
        [{"samples": control_samples, "color": (90, 90, 90), "label": "a(t)"}],
        "Control signal",
        "t [s]",
        "a [m/s^2]",
        (-150.0, 150.0),
    )
    return image


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "pid_position_stabilization_tuned.gif"
    output_path.parent.mkdir(exist_ok=True)
    params = PendulumParameters(damping=0.25)
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(dt=1.0 / 240.0, initial_phi=0.2, initial_phi_dot=0.0, max_abs_action=140.0),
    )
    controller = PositionPIDController(
        carrier_frequency=20.0,
        base_amplitude=86.5,
        kp=20.0,
        ki=3.8,
        kd=16.5,
        min_amplitude=62.5,
        max_amplitude=113.0,
    )
    frames: list[Image.Image] = []
    phi_samples: list[tuple[float, float]] = []
    amp_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    window: list[tuple[float, float, float]] = []
    stable_time = None
    frame_interval = 12

    for step in range(int(26.0 / simulator.config.dt)):
        state = simulator.step(controller)
        phi_samples.append((state.time_value, state.phi))
        amp_samples.append((state.time_value, controller.current_amplitude))
        control_samples.append((state.time_value, state.action))
        window.append((state.time_value, abs(state.phi), abs(state.phi_dot)))
        while window and state.time_value - window[0][0] > 1.5:
            window.pop(0)
        if step % frame_interval == 0:
            frames.append(
                draw_frame(
                    phi_samples,
                    amp_samples,
                    control_samples,
                    state.time_value,
                    state.phi,
                    state.action,
                    controller.current_amplitude,
                    params,
                    controller,
                )
            )
        if state.time_value > 6.0 and max(v[1] for v in window) < 0.10 and max(v[2] for v in window) < 0.40:
            stable_time = state.time_value
            break

    if stable_time is None:
        raise RuntimeError("Position PID controller did not stabilize within the simulation horizon.")

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
