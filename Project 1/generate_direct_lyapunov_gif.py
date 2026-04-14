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

from controllers.direct_lyapunov import DirectLyapunovController  # noqa: E402
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


def direct_lyapunov_value(phi: float, phi_dot: float, controller: DirectLyapunovController) -> float:
    return 0.5 * phi_dot * phi_dot + controller.stiffness_gain * (1.0 - math.cos(phi))


def draw_frame(
    phi_samples: list[tuple[float, float]],
    v_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_phi: float,
    current_action: float,
    controller: DirectLyapunovController,
    params: PendulumParameters,
) -> Image.Image:
    layout = standard_layout()
    width, height = layout["size"]
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.text((40, 20), "Direct Lyapunov stabilization on the exact vertical pendulum model", fill=(30, 45, 80), font=font)
    draw_pendulum_panel(draw, layout["left_panel"], layout["pivot"], int(layout["rod_px"] * params.length), current_phi, current_action)

    info_lines = [
        f"d = {params.damping:.2f}",
        "",
        "Green marker: upright target",
        "Orange bob: current pendulum state",
        "",
        "Direct control law:",
        "a = -g - l k_p - l k_c phi_dot sin(phi)",
        f"k_p = {controller.stiffness_gain:.2f}",
        f"k_c = {controller.damping_shaping_gain:.2f}",
        "",
        "Lyapunov function:",
        "V = 0.5 phi_dot^2 + k_p (1 - cos phi)",
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
    draw_axes_and_series(
        draw,
        layout["plot_mid"],
        [{"samples": v_samples, "color": (212, 100, 36), "label": "V_dir(t)"}],
        "Direct Lyapunov function",
        "t [s]",
        "V_dir",
        compute_positive_limits((value for _, value in v_samples), minimum_upper=0.12),
    )
    draw_axes_and_series(
        draw,
        layout["plot_bot"],
        [{"samples": control_samples, "color": (90, 90, 90), "label": "a(t)"}],
        "Control signal",
        "t [s]",
        "a [m/s^2]",
        compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=20.0),
    )
    return image


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "direct_lyapunov_stabilization_tuned.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = PendulumParameters(damping=0.25)
    controller = DirectLyapunovController(
        gravity=params.gravity,
        length=params.length,
        stiffness_gain=2.0,
        damping_shaping_gain=80.0,
    )
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(dt=1.0 / 240.0, initial_phi=0.2, initial_phi_dot=0.0, max_abs_action=140.0),
    )

    frames: list[Image.Image] = []
    phi_samples: list[tuple[float, float]] = []
    v_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    window: list[tuple[float, float, float]] = []
    stable_time = None
    # GIF timing is quantized effectively in centiseconds, so use an exact
    # 0.05 s frame step to keep playback 1:1 with simulation time.
    frame_interval = 12

    horizon = 15.0
    for step in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        value = direct_lyapunov_value(state.phi, state.phi_dot, controller)
        phi_samples.append((state.time_value, state.phi))
        v_samples.append((state.time_value, value))
        control_samples.append((state.time_value, state.action))
        window.append((state.time_value, abs(state.phi), abs(state.phi_dot)))
        while window and state.time_value - window[0][0] > 1.5:
            window.pop(0)

        if step % frame_interval == 0:
            frames.append(draw_frame(phi_samples, v_samples, control_samples, state.phi, state.action, controller, params))

        if stable_time is None and state.time_value > 1.0 and max(item[1] for item in window) < 0.10 and max(item[2] for item in window) < 0.40:
            stable_time = state.time_value

    if not frames:
        raise RuntimeError("No frames generated for direct Lyapunov GIF.")

    frame_duration_ms = int(1000.0 * simulator.config.dt * frame_interval)
    frames[0].save(output_path, save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0, disposal=2)
    if stable_time is None:
        print(f"Saved GIF to: {output_path}")
        print(f"Generated {len(frames)} frames over {simulator.state.time_value:.2f} s")
    else:
        print(f"Saved GIF to: {output_path}")
        print(f"Generated {len(frames)} frames over {simulator.state.time_value:.2f} s; first tube entry near t = {stable_time:.2f} s")


if __name__ == "__main__":
    main()
