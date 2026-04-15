from __future__ import annotations

import io
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib import patches
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.direct_orbit_tracking import DirectOrbitTrackingController  # noqa: E402
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


BG = "#f5f4f0"
PANEL = "#fcfaf5"
DARK = "#404045"


def compute_positive_limits(values, minimum_upper: float, margin_ratio: float = 0.12) -> tuple[float, float]:
    values = list(values)
    lower = min(0.0, min(values, default=0.0))
    upper = max(minimum_upper, max(values, default=minimum_upper))
    span = upper - lower
    pad = max(1e-9, span * margin_ratio)
    return lower - 0.15 * pad, upper + pad


def compute_symmetric_limits(values, minimum_half_range: float, margin_ratio: float = 0.12) -> tuple[float, float]:
    values = list(values)
    half_range = max(minimum_half_range, max((abs(v) for v in values), default=0.0))
    half_range *= 1.0 + margin_ratio
    return -half_range, half_range


def build_reference_orbit(controller: DirectOrbitTrackingController) -> list[tuple[float, float]]:
    reference_period = 2.0 * math.pi / controller.target_frequency
    orbit: list[tuple[float, float]] = []
    for index in range(1201):
        time_value = index * reference_period / 1200.0
        phi_ref, phi_dot_ref, _ = controller.reference_state(time_value)
        orbit.append((phi_ref, phi_dot_ref))
    return orbit


def _styled_axis(ax, title: str, xlabel: str, ylabel: str, y_limits: tuple[float, float]) -> None:
    ax.set_facecolor(PANEL)
    ax.set_title(title, fontsize=17, color="#1e2d50", pad=10)
    ax.set_xlabel(xlabel, fontsize=14)
    ax.set_ylabel(ylabel, fontsize=14)
    ax.set_ylim(*y_limits)
    ax.grid(True, alpha=0.28)
    ax.tick_params(labelsize=11)
    for spine in ax.spines.values():
        spine.set_color("#cccccc")


def _draw_fixed_pendulum(ax, phi: float, rod_length: float) -> None:
    ax.set_facecolor(PANEL)
    ax.set_aspect("equal")
    ax.set_xlim(-1.55 * rod_length, 1.55 * rod_length)
    ax.set_ylim(-2.0 * rod_length, 1.5 * rod_length)
    ax.axis("off")

    pivot_x = 0.0
    pivot_y = 0.0
    bob_x = rod_length * math.sin(phi)
    bob_y = rod_length * math.cos(phi)

    ax.plot([-0.9 * rod_length, 0.9 * rod_length], [pivot_y, pivot_y], color="#5f6064", linewidth=6)
    ax.plot([pivot_x, pivot_x], [pivot_y, pivot_y + rod_length], color="#b6d4b6", linewidth=2)
    ax.add_patch(patches.Circle((pivot_x, pivot_y), 0.05 * rod_length, color="#444444"))
    ax.add_patch(patches.Circle((pivot_x, pivot_y + rod_length), 0.04 * rod_length, color="#50AF50", alpha=0.9))
    ax.plot([pivot_x, bob_x], [pivot_y, bob_y], color="#2662AA", linewidth=5)
    ax.add_patch(patches.Circle((bob_x, bob_y), 0.11 * rod_length, facecolor="#D46424", edgecolor="#783214", linewidth=1.5))


def render_frame(
    phi_samples: list[tuple[float, float]],
    tracking_error_samples: list[tuple[float, float]],
    control_samples: list[tuple[float, float]],
    phase_samples: list[tuple[float, float]],
    reference_orbit: list[tuple[float, float]],
    current_phi: float,
    controller: DirectOrbitTrackingController,
    params: PendulumParameters,
) -> Image.Image:
    fig = plt.figure(figsize=(16, 10), dpi=140, facecolor=BG)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.65], height_ratios=[1, 1, 1])
    fig.subplots_adjust(left=0.045, right=0.985, bottom=0.055, top=0.885, wspace=0.15, hspace=0.34)
    fig.suptitle("Direct orbit tracking from nominal start", fontsize=24, fontweight="bold", color="#1e2d50", y=0.97)

    ax_left = fig.add_subplot(gs[:, 0])
    _draw_fixed_pendulum(ax_left, current_phi, params.length)
    info_lines = [
        f"d = {params.damping:.2f}",
        "",
        "Fixed suspension frame:",
        "no vertical support motion is shown",
        "",
        "Target orbit:",
        f"A* = {controller.target_amplitude:.2f} rad",
        f"w* = {controller.target_frequency:.2f} rad/s",
        "",
        "Direct control benchmark",
        "for orbital tracking.",
    ]
    ax_left.text(
        0.05,
        0.02,
        "\n".join(info_lines),
        transform=ax_left.transAxes,
        fontsize=13,
        va="bottom",
        ha="left",
        color=DARK,
        linespacing=1.15,
    )

    ax_phase = fig.add_subplot(gs[0, 1])
    ax_err = fig.add_subplot(gs[1, 1])
    ax_u = fig.add_subplot(gs[2, 1])

    phi_lim = 1.35 * controller.target_amplitude
    phi_dot_lim = 1.45 * controller.target_amplitude * controller.target_frequency
    ax_phase.set_facecolor(PANEL)
    ax_phase.set_title("Phase portrait", fontsize=17, color="#1e2d50", pad=10)
    ax_phase.set_xlabel("phi [rad]", fontsize=14)
    ax_phase.set_ylabel("phi_dot [rad/s]", fontsize=14)
    ax_phase.set_xlim(-phi_lim, phi_lim)
    ax_phase.set_ylim(-phi_dot_lim, phi_dot_lim)
    ax_phase.grid(True, alpha=0.28)
    ax_phase.tick_params(labelsize=11)
    for spine in ax_phase.spines.values():
        spine.set_color("#cccccc")
    if reference_orbit:
        ax_phase.plot(
            [phi for phi, _ in reference_orbit],
            [phi_dot for _, phi_dot in reference_orbit],
            color="#A3BE8C",
            linewidth=2.0,
            label="reference orbit",
        )
    if phase_samples:
        ax_phase.plot(
            [phi for phi, _ in phase_samples],
            [phi_dot for _, phi_dot in phase_samples],
            color="#5E81AC",
            linewidth=2.0,
            label="trajectory",
        )
        ax_phase.scatter([phase_samples[-1][0]], [phase_samples[-1][1]], color="#BF616A", s=30, zorder=4)
    ax_phase.legend(loc="upper right", fontsize=10, frameon=True)

    _styled_axis(
        ax_err,
        "Tracking error",
        "t [s]",
        "||e||",
        compute_positive_limits((value for _, value in tracking_error_samples), minimum_upper=0.03),
    )
    if tracking_error_samples:
        ax_err.plot(
            [x for x, _ in tracking_error_samples],
            [y for _, y in tracking_error_samples],
            color="#D46424",
            linewidth=2.0,
            label="||e(t)||",
        )
        ax_err.legend(loc="upper right", fontsize=10, frameon=True)

    _styled_axis(
        ax_u,
        "Control signal",
        "t [s]",
        "a [m/s^2]",
        compute_symmetric_limits((value for _, value in control_samples), minimum_half_range=20.0),
    )
    if control_samples:
        ax_u.plot(
            [x for x, _ in control_samples],
            [y for _, y in control_samples],
            color="#5E81AC",
            linewidth=2.0,
            label="a(t)",
        )
        ax_u.legend(loc="upper right", fontsize=10, frameon=True)

    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", facecolor=fig.get_facecolor())
    plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert("P", palette=Image.ADAPTIVE)


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "direct_orbit_tracking_nominal.gif"
    output_path.parent.mkdir(exist_ok=True)

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
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(
            dt=1.0 / 240.0,
            initial_phi=0.0,
            initial_phi_dot=controller.target_amplitude * controller.target_frequency,
            max_abs_action=controller.max_abs_action,
        ),
    )

    reference_orbit = build_reference_orbit(controller)
    phi_samples: list[tuple[float, float]] = []
    tracking_error_samples: list[tuple[float, float]] = []
    control_samples: list[tuple[float, float]] = []
    phase_samples: list[tuple[float, float]] = []
    frames: list[Image.Image] = []

    frame_interval = 12
    horizon = 15.0
    for step in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        tracking_error = controller.tracking_error(state.time_value, (state.phi, state.phi_dot))[2]
        phi_samples.append((state.time_value, state.phi))
        tracking_error_samples.append((state.time_value, tracking_error))
        control_samples.append((state.time_value, state.action))
        phase_samples.append((state.phi, state.phi_dot))

        if step % frame_interval == 0:
            frames.append(
                render_frame(
                    phi_samples,
                    tracking_error_samples,
                    control_samples,
                    phase_samples,
                    reference_orbit,
                    state.phi,
                    controller,
                    params,
                )
            )

    if not frames:
        raise RuntimeError("No frames generated for direct_orbit_tracking_nominal.gif")

    frame_duration_ms = int(1000.0 * simulator.config.dt * frame_interval)
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=frame_duration_ms,
        loop=0,
        disposal=2,
    )
    print(f"Saved GIF to: {output_path}")


if __name__ == "__main__":
    main()
