from __future__ import annotations

import math
import sys
from pathlib import Path

from PIL import Image
from matplotlib_gif_utils import (
    compute_positive_limits,
    compute_symmetric_limits,
    render_standard_frame,
)

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.pid import CycleEnergyPDController
from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters


def cycle_energy_value(phi: float, phi_dot: float, weight: float) -> float:
    return 0.5 * phi_dot * phi_dot + weight * (1.0 - math.cos(phi))


def draw_frame(
    phi_samples: list[tuple[float, float]],
    energy_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_phi: float,
    params: PendulumParameters,
    controller: CycleEnergyPDController,
) -> Image.Image:
    lines = [
        "",
        "Law:",
        "alpha = base + PD(E_cycle)",
        "a(t) = alpha cos(omega t)",
        "",
        f"kp = {controller.kp:.1f}",
        f"kd = {controller.kd:.1f}",
    ]
    return render_standard_frame(
        title="PD cycle-energy controller for the Kapitza pendulum",
        current_phi=current_phi,
        rod_length=params.length,
        info_lines=lines,
        plot_defs=[
            {
                "series": [{"samples": phi_samples, "color": "#2662AA", "label": "phi(t)"}],
                "title": "Angle deviation",
                "x_label": "t [s]",
                "y_label": "phi [rad]",
                "y_limits": compute_symmetric_limits((value for _, value in phi_samples), minimum_half_range=0.15),
            },
            {
                "series": [{"samples": energy_samples, "color": "#D46424", "label": "E_cycle"}],
                "title": "Cycle energy proxy",
                "x_label": "t [s]",
                "y_label": "E_cycle",
                "y_limits": compute_positive_limits((value for _, value in energy_samples), minimum_upper=0.30),
            },
            {
                "series": [{"samples": control_samples, "color": "#606060", "label": "a(t)"}],
                "title": "Control signal",
                "x_label": "t [s]",
                "y_label": "a [m/s^2]",
                "y_limits": compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=60.0),
            },
        ],
    )


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "pid_cycle_energy_stabilization_tuned.gif"
    output_path.parent.mkdir(exist_ok=True)
    params = PendulumParameters(damping=0.25)
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(dt=1.0 / 240.0, initial_phi=0.2, initial_phi_dot=0.0, max_abs_action=140.0),
    )
    controller = CycleEnergyPDController(
        carrier_frequency=14.0,
        base_amplitude=82.0,
        kp=24.0,
        kd=7.5,
        energy_weight=6.3,
        min_amplitude=68.5,
        max_amplitude=133.5,
    )
    frames: list[Image.Image] = []
    phi_samples: list[tuple[float, float]] = []
    energy_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    window: list[tuple[float, float, float]] = []
    stable_time = None
    frame_interval = 12

    for step in range(int(24.0 / simulator.config.dt)):
        state = simulator.step(controller)
        phi_samples.append((state.time_value, state.phi))
        energy_samples.append((state.time_value, cycle_energy_value(state.phi, state.phi_dot, controller.energy_weight)))
        control_samples.append((state.time_value, state.action))
        window.append((state.time_value, abs(state.phi), abs(state.phi_dot)))
        while window and state.time_value - window[0][0] > 1.5:
            window.pop(0)
        if step % frame_interval == 0:
            frames.append(
                draw_frame(
                    phi_samples,
                    energy_samples,
                    control_samples,
                    state.phi,
                    params,
                    controller,
                )
            )
        if state.time_value > 6.0 and max(v[1] for v in window) < 0.10 and max(v[2] for v in window) < 0.40:
            stable_time = state.time_value
            break

    if stable_time is None:
        raise RuntimeError("Cycle-energy PD controller did not stabilize within the simulation horizon.")

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
