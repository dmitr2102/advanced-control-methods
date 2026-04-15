from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers import (  # noqa: E402
    AveragedEnergyController,
    CycleEnergyPDController,
    HarmonicController,
    LyapunovController,
)
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


ANGLE_TUBE = 0.10
RATE_TUBE = 0.40
HORIZON = 15.0
DT = 1.0 / 240.0


@dataclass
class MethodResult:
    label: str
    color: str
    times: list[float]
    angle_error: list[float]
    convergence_time: float | None
    convergence_error: float | None
    tube_percent: float | None


def make_controllers() -> list[tuple[str, object, str]]:
    return [
        ("Harmonic", HarmonicController(), "#2662AA"),
        ("Averaged-energy", AveragedEnergyController(), "#D46424"),
        ("Lyapunov CLF", LyapunovController(), "#3C965A"),
        ("PD cycle-energy", CycleEnergyPDController(), "#BE4646"),
    ]


def simulate_method(label: str, controller: object, color: str) -> MethodResult:
    params = PendulumParameters(damping=0.25)
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(dt=DT, initial_phi=0.2, initial_phi_dot=0.0, max_abs_action=140.0),
    )
    if hasattr(controller, "reset"):
        controller.reset()

    times: list[float] = [0.0]
    phis: list[float] = [simulator.state.phi]
    phi_dots: list[float] = [simulator.state.phi_dot]

    steps = int(HORIZON / DT)
    for _ in range(steps):
        state = simulator.step(controller)
        times.append(state.time_value)
        phis.append(state.phi)
        phi_dots.append(state.phi_dot)

    # For the angle-error plot, mark the first sample after the last exit from the
    # angle tube. This matches the visual interpretation of the figure itself.
    convergence_idx = 0
    for idx in range(len(times) - 1, -1, -1):
        if abs(phis[idx]) > ANGLE_TUBE:
            convergence_idx = min(idx + 1, len(times) - 1)
            break

    convergence_time = times[convergence_idx]
    convergence_error = abs(phis[convergence_idx])
    tube_percent = 100.0 * convergence_error / ANGLE_TUBE

    return MethodResult(
        label=label,
        color=color,
        times=times,
        angle_error=[abs(value) for value in phis],
        convergence_time=convergence_time,
        convergence_error=convergence_error,
        tube_percent=tube_percent,
    )


def main() -> None:
    results = [simulate_method(label, controller, color) for label, controller, color in make_controllers()]

    figures_dir = PROJECT_ROOT / "figures"
    figures_dir.mkdir(exist_ok=True)
    output_png = figures_dir / "tuned_controller_angle_error_comparison_15s.png"
    output_pdf = figures_dir / "tuned_controller_angle_error_comparison_15s.pdf"
    output_json = figures_dir / "tuned_controller_angle_error_comparison_15s.json"

    y_max = max(0.22, max((max(result.angle_error) for result in results), default=0.22)) * 1.05

    fig, ax = plt.subplots(figsize=(11.5, 6.6), constrained_layout=True)
    for result in results:
        ax.plot(result.times, result.angle_error, linewidth=1.15, color=result.color, label=result.label)
        if result.convergence_time is not None and result.convergence_error is not None:
            ax.scatter(
                [result.convergence_time],
                [result.convergence_error],
                s=42,
                color=result.color,
                edgecolors="white",
                linewidths=0.8,
                zorder=5,
            )
            ax.axvline(result.convergence_time, color=result.color, linewidth=2.6, alpha=0.35)

    ax.axhline(ANGLE_TUBE, color="#666666", linestyle="--", linewidth=1.2, label="Tube boundary")
    ax.set_xlim(0.0, HORIZON)
    ax.set_ylim(0.0, y_max)
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"$|\varphi(t)|$ [rad]")
    ax.set_title("Angle-error comparison for tuned Kapitza pendulum controllers")
    ax.grid(True, alpha=0.28)
    ax.legend(loc="upper right", frameon=True)

    summary_lines: list[str] = []
    for result in results:
        if result.convergence_time is None or result.tube_percent is None:
            summary_lines.append(f"{result.label}: no convergence by {HORIZON:.1f} s")
        else:
            summary_lines.append(f"{result.label}: t_conv = {result.convergence_time:.3f} s")

    summary_text = "\n".join(summary_lines)
    ax.text(
        0.015,
        0.98,
        summary_text,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "#cccccc", "alpha": 0.9},
    )

    fig.savefig(output_png, dpi=220)
    fig.savefig(output_pdf)
    plt.close(fig)

    output_json.write_text(
        json.dumps(
            {
                "horizon_s": HORIZON,
                "dt": DT,
                "angle_tube_rad": ANGLE_TUBE,
                "rate_tube_rad_s": RATE_TUBE,
                "methods": [asdict(result) for result in results],
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"Saved comparison figure to: {output_png}")
    print(f"Saved comparison figure to: {output_pdf}")
    print(f"Saved summary data to: {output_json}")
    for line in summary_lines:
        print(line)


if __name__ == "__main__":
    main()
