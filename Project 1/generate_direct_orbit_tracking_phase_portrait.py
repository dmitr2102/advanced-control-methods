from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.direct_orbit_tracking import DirectOrbitTrackingController  # noqa: E402
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


def simulate_trajectory(
    controller: DirectOrbitTrackingController,
    params: PendulumParameters,
    initial_phi: float,
    initial_phi_dot: float,
    horizon: float,
) -> tuple[list[float], list[float], list[float]]:
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(
            dt=1.0 / 240.0,
            initial_phi=initial_phi,
            initial_phi_dot=initial_phi_dot,
            max_abs_action=controller.max_abs_action,
        ),
    )

    times: list[float] = []
    phi_values: list[float] = []
    phi_dot_values: list[float] = []
    for _ in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        times.append(state.time_value)
        phi_values.append(state.phi)
        phi_dot_values.append(state.phi_dot)
    return times, phi_values, phi_dot_values


def main() -> None:
    figures_dir = PROJECT_ROOT / "figures"
    figures_dir.mkdir(exist_ok=True)

    output_png = figures_dir / "direct_orbit_tracking_phase_portrait_30s.png"
    output_pdf = figures_dir / "direct_orbit_tracking_phase_portrait_30s.pdf"
    output_json = figures_dir / "direct_orbit_tracking_phase_portrait_30s.json"

    params = PendulumParameters(damping=0.18)
    controller = DirectOrbitTrackingController(
        gravity=params.gravity,
        length=params.length,
        damping=params.damping,
        target_amplitude=0.14,
        target_frequency=1.45,
        position_gain=12.0,
        rate_gain=4.0,
        orbit_gain=10.0,
        phase_gain=0.0,
        regularization_eps=0.08,
        max_abs_action=140.0,
    )
    horizon = 30.0
    nominal_time, nominal_phi, nominal_phi_dot = simulate_trajectory(
        controller,
        params,
        initial_phi=0.0,
        initial_phi_dot=controller.target_amplitude * controller.target_frequency,
        horizon=horizon,
    )

    outside_time, outside_phi, outside_phi_dot = simulate_trajectory(
        controller,
        params,
        initial_phi=0.22,
        initial_phi_dot=0.0,
        horizon=horizon,
    )

    inside_time, inside_phi, inside_phi_dot = simulate_trajectory(
        controller,
        params,
        initial_phi=0.05,
        initial_phi_dot=0.0,
        horizon=horizon,
    )

    reference_period = 2.0 * math.pi / controller.target_frequency
    reference_times = [index * reference_period / 1200.0 for index in range(1201)]
    reference_phi: list[float] = []
    reference_phi_dot: list[float] = []
    for time_value in reference_times:
        phi_ref, phi_dot_ref, _ = controller.reference_state(time_value)
        reference_phi.append(phi_ref)
        reference_phi_dot.append(phi_dot_ref)

    split_index = int(10.0 / (1.0 / 240.0))
    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    ax.plot(reference_phi, reference_phi_dot, color="#a3be8c", linewidth=2.0, linestyle="--", label="reference orbit")
    ax.plot(nominal_phi[:split_index], nominal_phi_dot[:split_index], color="#d08770", linewidth=1.4, alpha=0.9, label="nominal transient")
    ax.plot(outside_phi, outside_phi_dot, color="#b48ead", linewidth=1.4, alpha=0.9, label="outside-start trajectory")
    ax.plot(inside_phi, inside_phi_dot, color="#ebcb8b", linewidth=1.4, alpha=0.95, label="inside-start trajectory")
    ax.scatter([nominal_phi[0]], [nominal_phi_dot[0]], color="#bf616a", s=55, zorder=5, label="nominal start")
    ax.scatter([outside_phi[0]], [outside_phi_dot[0]], color="#8f5aa8", s=55, zorder=5, label="outside start")
    ax.scatter([inside_phi[0]], [inside_phi_dot[0]], color="#c7921b", s=55, zorder=5, label="inside start")
    ax.scatter([nominal_phi[-1]], [nominal_phi_dot[-1]], color="#2e3440", s=55, zorder=5, label="nominal end")
    ax.set_title("Direct Orbital Tracking Controller: Phase Portrait over 30 s")
    ax.set_xlabel(r"$\varphi$ [rad]")
    ax.set_ylabel(r"$\dot{\varphi}$ [rad/s]")
    ax.grid(True, alpha=0.28)
    ax.legend(loc="upper right")

    fig.savefig(output_png, dpi=220)
    fig.savefig(output_pdf)
    plt.close(fig)

    output_json.write_text(
        json.dumps(
            {
                "horizon_s": horizon,
                "dt": 1.0 / 240.0,
                "nominal": {
                    "times": nominal_time,
                    "phi": nominal_phi,
                    "phi_dot": nominal_phi_dot,
                    "transient_end_time_s": 10.0,
                },
                "outside_start": {
                    "times": outside_time,
                    "phi": outside_phi,
                    "phi_dot": outside_phi_dot,
                    "initial_phi": 0.22,
                    "initial_phi_dot": 0.0,
                },
                "inside_start": {
                    "times": inside_time,
                    "phi": inside_phi,
                    "phi_dot": inside_phi_dot,
                    "initial_phi": 0.05,
                    "initial_phi_dot": 0.0,
                },
                "reference_phi": reference_phi,
                "reference_phi_dot": reference_phi_dot,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"Saved figure to: {output_png}")
    print(f"Saved figure to: {output_pdf}")
    print(f"Saved data to: {output_json}")


if __name__ == "__main__":
    main()
