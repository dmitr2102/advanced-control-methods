from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image
from matplotlib_gif_utils import compute_symmetric_limits, render_standard_frame

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.harmonic import HarmonicController
from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters


def draw_frame(
    phi_samples: list[tuple[float, float]],
    freq_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_phi: float,
    current_action: float,
    controller: HarmonicController,
    params: PendulumParameters,
) -> Image.Image:
    info_lines = [
        "Constant harmonic excitation",
        "",
        f"A = {controller.amplitude:.1f} m/s^2",
        f"omega = {controller.frequency:.1f} rad/s",
        f"d = {params.damping:.2f}",
        "",
        "Green marker: upright target",
        "Orange bob: current pendulum state",
    ]
    return render_standard_frame(
        title="Tuned harmonic controller for the Kapitza pendulum",
        current_phi=current_phi,
        rod_length=params.length,
        info_lines=info_lines,
        plot_defs=[
            {
                "series": [{"samples": phi_samples, "color": "#2662AA", "label": "phi(t)"}],
                "title": "Angle deviation",
                "x_label": "t [s]",
                "y_label": "phi [rad]",
                "y_limits": compute_symmetric_limits((value for _, value in phi_samples), minimum_half_range=0.15),
            },
            {
                "series": [{"samples": freq_samples, "color": "#D46424", "label": "omega"}],
                "title": "Carrier frequency",
                "x_label": "t [s]",
                "y_label": "omega [rad/s]",
                "y_limits": (controller.frequency - 1.0, controller.frequency + 1.0),
            },
            {
                "series": [{"samples": control_samples, "color": "#606060", "label": "a(t)"}],
                "title": "Control signal",
                "x_label": "t [s]",
                "y_label": "a [m/s^2]",
                "y_limits": compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=70.0),
            },
        ],
    )


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "harmonic_stabilization_tuned.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = PendulumParameters(damping=0.25)
    controller = HarmonicController(amplitude=116.5, frequency=25.3)
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(dt=1.0 / 240.0, initial_phi=0.2, initial_phi_dot=0.0, max_abs_action=140.0),
    )

    frames: list[Image.Image] = []
    phi_samples: list[tuple[float, float]] = []
    freq_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    window: list[tuple[float, float, float]] = []
    stable_time = None
    frame_interval = 12

    for step in range(int(20.0 / simulator.config.dt)):
        state = simulator.step(controller)
        phi_samples.append((state.time_value, state.phi))
        freq_samples.append((state.time_value, controller.frequency))
        control_samples.append((state.time_value, state.action))
        window.append((state.time_value, abs(state.phi), abs(state.phi_dot)))
        while window and state.time_value - window[0][0] > 1.5:
            window.pop(0)
        if step % frame_interval == 0:
            frames.append(draw_frame(phi_samples, freq_samples, control_samples, state.phi, state.action, controller, params))
        if state.time_value > 2.0 and max(v[1] for v in window) < 0.10 and max(v[2] for v in window) < 0.40:
            stable_time = state.time_value
            break

    if stable_time is None:
        raise RuntimeError("Tuned harmonic controller did not stabilize within the simulation horizon.")

    frame_duration_ms = int(1000.0 * simulator.config.dt * frame_interval)
    frames[0].save(output_path, save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0, disposal=2)
    print(f"Saved GIF to: {output_path}")
    print(f"Generated {len(frames)} frames up to t = {stable_time:.2f} s")


if __name__ == "__main__":
    main()
