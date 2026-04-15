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

from controllers.adaptive_limit_cycle_lyapunov import AdaptiveLimitCycleLyapunovController  # noqa: E402
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


def draw_frame(
    lyapunov_samples: list[tuple[float, float]],
    tracking_error_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    current_phi: float,
    current_alpha: float,
    current_omega: float,
    controller: AdaptiveLimitCycleLyapunovController,
    params: PendulumParameters,
) -> Image.Image:
    info_lines = [
        f"d = {params.damping:.2f}",
        "",
        "Target oscillation:",
        f"A* = {controller.target_amplitude:.2f} rad",
        f"w* = {controller.target_frequency:.2f} rad/s",
        "",
        "Adaptive carrier law:",
        "a(t) = alpha(s,t) cos(psi(t))",
        "psi_dot = omega(s,t)",
        f"omega base = {controller.base_carrier_frequency:.2f} rad/s",
        "",
        "Tracking/Lyapunov gains:",
        f"kp = {controller.position_gain:.2f}",
        f"kd = {controller.rate_gain:.2f}",
        f"kE = {controller.energy_gain:.2f}",
    ]
    return render_standard_frame(
        title="Adaptive Lyapunov limit-cycle control with variable carrier amplitude and frequency",
        current_phi=current_phi,
        rod_length=params.length,
        info_lines=info_lines,
        title_fontsize=22,
        info_y=0.02,
        info_fontsize=13,
        pendulum_y_limits=(-2.45 * params.length, 1.15 * params.length),
        plot_defs=[
            {
                "series": [{"samples": lyapunov_samples, "color": "#606060", "label": "V(t)"}],
                "title": "Lyapunov function",
                "x_label": "t [s]",
                "y_label": "V",
                "y_limits": compute_positive_limits((value for _, value in lyapunov_samples), minimum_upper=0.05),
            },
            {
                "series": [{"samples": tracking_error_samples, "color": "#D46424", "label": "||e(t)||"}],
                "title": "Tracking error",
                "x_label": "t [s]",
                "y_label": "||e||",
                "y_limits": compute_positive_limits((value for _, value in tracking_error_samples), minimum_upper=0.05),
            },
            {
                "series": [{"samples": control_samples, "color": "#2662AA", "label": "a(t)"}],
                "title": "Control signal",
                "x_label": "t [s]",
                "y_label": "a [m/s^2]",
                "y_limits": compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=20.0),
            },
        ],
    )


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "adaptive_limit_cycle_lyapunov_tracking.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = PendulumParameters(damping=0.18)
    controller = AdaptiveLimitCycleLyapunovController(
        gravity=params.gravity,
        length=params.length,
        damping=params.damping,
        target_amplitude=0.14,
        target_frequency=1.45,
        position_gain=12.0,
        rate_gain=5.0,
        energy_gain=10.0,
        energy_weight=3.0,
        base_carrier_frequency=20.0,
        angle_frequency_gain=8.0,
        rate_frequency_gain=0.5,
        energy_frequency_gain=10.0,
        min_carrier_frequency=16.0,
        max_carrier_frequency=28.0,
        stiffness_floor=0.5,
        stiffness_ceiling=12.0,
        regularization_eps=0.05,
        min_amplitude=68.0,
        max_amplitude=128.0,
    )
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(
            dt=1.0 / 240.0,
            initial_phi=0.0,
            initial_phi_dot=controller.target_amplitude * controller.target_frequency,
            max_abs_action=140.0,
        ),
    )

    frames: list[Image.Image] = []
    lyapunov_samples: list[tuple[float, float]] = []
    tracking_error_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    frame_interval = 12
    horizon = 15.0

    for step in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        tracking_error = controller.tracking_error(state.time_value, (state.phi, state.phi_dot))[2]
        lyapunov_value = controller.lyapunov_value(state.time_value, (state.phi, state.phi_dot))

        lyapunov_samples.append((state.time_value, lyapunov_value))
        tracking_error_samples.append((state.time_value, tracking_error))
        control_samples.append((state.time_value, state.action))

        if step % frame_interval == 0:
            frames.append(
                draw_frame(
                    lyapunov_samples,
                    tracking_error_samples,
                    control_samples,
                    state.phi,
                    controller.last_amplitude,
                    controller.last_carrier_frequency,
                    controller,
                    params,
                )
            )

    if not frames:
        raise RuntimeError("No frames generated for the adaptive limit-cycle Lyapunov GIF.")

    frame_duration_ms = int(1000.0 * simulator.config.dt * frame_interval)
    frames[0].save(output_path, save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0, disposal=2)
    print(f"Saved GIF to: {output_path}")
    print(
        "Generated "
        f"{len(frames)} frames over {simulator.state.time_value:.2f} s; "
        f"final normalized tracking error = {tracking_error_samples[-1][1]:.4f}"
    )


if __name__ == "__main__":
    main()
