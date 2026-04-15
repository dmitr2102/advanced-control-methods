from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.adaptive_limit_cycle_lyapunov import AdaptiveLimitCycleLyapunovController  # noqa: E402
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


def main() -> None:
    figures_dir = PROJECT_ROOT / "figures"
    figures_dir.mkdir(exist_ok=True)

    output_png = figures_dir / "adaptive_limit_cycle_phase_portrait_30s.png"
    output_pdf = figures_dir / "adaptive_limit_cycle_phase_portrait_30s.pdf"
    output_json = figures_dir / "adaptive_limit_cycle_phase_portrait_30s.json"

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

    horizon = 30.0
    times: list[float] = []
    phi_values: list[float] = []
    phi_dot_values: list[float] = []
    phi_ref_values: list[float] = []
    phi_dot_ref_values: list[float] = []

    for _ in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        phi_ref, phi_dot_ref, _ = controller.reference_state(state.time_value)
        times.append(state.time_value)
        phi_values.append(state.phi)
        phi_dot_values.append(state.phi_dot)
        phi_ref_values.append(phi_ref)
        phi_dot_ref_values.append(phi_dot_ref)

    reference_period = 2.0 * math.pi / controller.target_frequency
    reference_times = [
        index * reference_period / 1200.0
        for index in range(1201)
    ]
    reference_phi_closed: list[float] = []
    reference_phi_dot_closed: list[float] = []
    for time_value in reference_times:
        phi_ref, phi_dot_ref, _ = controller.reference_state(time_value)
        reference_phi_closed.append(phi_ref)
        reference_phi_dot_closed.append(phi_dot_ref)

    fig, ax = plt.subplots(figsize=(10.8, 8), constrained_layout=True)
    ax.plot(reference_phi_closed, reference_phi_dot_closed, color="#a3be8c", linewidth=2.0, linestyle="--", label="reference orbit")

    points = list(zip(phi_values, phi_dot_values))
    segments = [[points[i], points[i + 1]] for i in range(len(points) - 1)]
    norm = Normalize(vmin=times[0], vmax=times[-1])
    collection = LineCollection(
        segments,
        cmap="plasma",
        norm=norm,
        linewidth=1.2,
        alpha=0.95,
    )
    collection.set_array(times[:-1])
    ax.add_collection(collection)

    ax.scatter([phi_values[0]], [phi_dot_values[0]], color="#bf616a", s=55, zorder=5, label="start")
    ax.scatter([phi_values[-1]], [phi_dot_values[-1]], color="#2e3440", s=55, zorder=5, label="end")

    ax.set_title("Adaptive Limit-Cycle Controller: Phase Portrait over 30 s")
    ax.set_xlabel(r"$\varphi$ [rad]")
    ax.set_ylabel(r"$\dot{\varphi}$ [rad/s]")
    ax.grid(True, alpha=0.28)
    ax.legend(loc="upper right")
    colorbar = fig.colorbar(collection, ax=ax, pad=0.02)
    colorbar.set_label("time [s]")

    fig.savefig(output_png, dpi=220)
    fig.savefig(output_pdf)
    plt.close(fig)

    output_json.write_text(
        json.dumps(
            {
                "horizon_s": horizon,
                "dt": simulator.config.dt,
                "times": times,
                "phi": phi_values,
                "phi_dot": phi_dot_values,
                "phi_ref": phi_ref_values,
                "phi_dot_ref": phi_dot_ref_values,
                "reference_period": reference_period,
                "reference_phi_closed": reference_phi_closed,
                "reference_phi_dot_closed": reference_phi_dot_closed,
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
