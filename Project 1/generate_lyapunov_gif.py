from __future__ import annotations

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

from controllers.lyapunov import LyapunovController
from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters


def lyapunov_value(phi: float, phi_dot: float, controller: LyapunovController) -> float:
    kinetic = 0.5 * phi_dot * phi_dot
    quadratic = 0.5 * controller.stiffness_linear * (phi ** 2)
    quartic = 0.25 * controller.stiffness_cubic * (phi ** 4)
    return kinetic + quadratic + quartic


def draw_frame(
    phi_samples: list[tuple[float, float]],
    v_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_phi: float,
    params: PendulumParameters,
) -> Image.Image:
    info_lines = [
        f"d = {params.damping: .2f}",
        "",
        "Green marker: upright target",
        "Orange bob: current pendulum state",
        "",
        "Control law:",
        "a(t) = alpha(phi) cos(omega t)",
        "alpha(phi) is chosen so that",
        "the local averaged target dynamics",
        "has a polynomial Lyapunov function.",
    ]
    return render_standard_frame(
        title="Lyapunov-based stabilization of the Kapitza pendulum",
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
                "series": [{"samples": v_samples, "color": "#D46424", "label": "V_cl(t)"}],
                "title": "Lyapunov function",
                "x_label": "t [s]",
                "y_label": "V_cl",
                "y_limits": compute_positive_limits((value for _, value in v_samples), minimum_upper=0.25),
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
    output_path = PROJECT_ROOT / "animations" / "lyapunov_stabilization_tuned.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = PendulumParameters(damping=0.25)
    plant = KapitzaPendulumPlant(params)
    controller = LyapunovController(
        gravity=params.gravity,
        length=params.length,
        carrier_frequency=18.0,
        stiffness_linear=0.35,
        stiffness_cubic=2.35,
        min_amplitude=62.5,
        max_amplitude=104.0,
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
    max_steps = int(22.0 / simulator.config.dt)

    for step in range(max_steps):
        state = simulator.step(controller)
        value = lyapunov_value(state.phi, state.phi_dot, controller)
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
                    current_phi=state.phi,
                    params=params,
                )
            )

        if state.time_value > 6.0 and max(v[1] for v in window) < 0.10 and max(v[2] for v in window) < 0.40:
            stable_time = state.time_value
            break

    if stable_time is None:
        raise RuntimeError("Lyapunov controller did not stabilize the pendulum within the simulation horizon.")

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
