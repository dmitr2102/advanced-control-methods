from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from gif_plot_utils import (
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

from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters


@dataclass
class RampHarmonicController:
    amplitude: float = 100.0
    omega_start: float = 15.0
    omega_end: float = 18.0
    ramp_duration: float = 6.0
    phase: float = 0.0
    last_time_value: float | None = None
    current_omega: float = 15.0
    name: str = "harmonic_frequency_ramp"

    def compute_action(self, time_value: float, state: tuple[float, float]):
        del state
        if self.last_time_value is None:
            self.last_time_value = time_value

        dt = time_value - self.last_time_value
        self.last_time_value = time_value

        tau = min(1.0, max(0.0, time_value / self.ramp_duration))
        smooth_tau = tau * tau * (3.0 - 2.0 * tau)
        self.current_omega = self.omega_start + (self.omega_end - self.omega_start) * smooth_tau
        self.phase += self.current_omega * dt

        class Output:
            def __init__(self, action: float, name: str) -> None:
                self.action = action
                self.name = name

        return Output(
            action=self.amplitude * math.cos(self.phase),
            name=self.name,
        )


def draw_frame(
    samples: list[tuple[float, float, float, float, float]],
    current_time: float,
    current_phi: float,
    current_action: float,
    current_omega: float,
    params: PendulumParameters,
    image_size: tuple[int, int] = (1400, 920),
) -> Image.Image:
    layout = standard_layout()
    width, height = image_size
    image = Image.new("RGB", image_size, (245, 244, 240))
    draw = ImageDraw.Draw(image)
    title_font = ImageFont.load_default()
    text_font = ImageFont.load_default()

    draw.text((60, 55), "Kapitza pendulum with frequency ramp", fill=(30, 45, 80), font=title_font)
    draw_pendulum_panel(draw, layout["left_panel"], layout["pivot"], int(layout["rod_px"] * params.length), current_phi, current_action)

    info_lines = [
        f"omega = {current_omega: .2f} rad/s",
        "",
        "Ramp law:",
        "omega(t) increases smoothly",
        "until the pendulum settles",
        "near the upright equilibrium.",
    ]
    draw_info_lines(draw, layout["info_x"], layout["info_y"], info_lines, layout["info_step"])

    draw_axes_and_series(
        draw,
        layout["plot_top"],
        [{"samples": [(time_value, phi_value) for time_value, phi_value, _, _, _ in samples], "color": (38, 98, 170), "label": "phi(t)"}],
        "Angle deviation",
        "t [s]",
        "phi [rad]",
        compute_symmetric_limits((phi_value for _, phi_value, _, _, _ in samples), minimum_half_range=0.15),
    )
    draw_axes_and_series(
        draw,
        layout["plot_mid"],
        [{"samples": [(time_value, omega_value) for time_value, _, _, _, omega_value in samples], "color": (212, 100, 36), "label": "omega(t)"}],
        "Frequency ramp",
        "t [s]",
        "omega [rad/s]",
        (14.0, 19.0),
    )
    draw_axes_and_series(
        draw,
        layout["plot_bot"],
        [{"samples": [(time_value, action_value) for time_value, _, _, action_value, _ in samples], "color": (90, 90, 90), "label": "a(t)"}],
        "Control signal",
        "t [s]",
        "a [m/s^2]",
        compute_symmetric_limits((action_value for _, _, action_value, _, _ in samples), minimum_half_range=60.0),
    )

    return image


def has_stabilized(window: list[tuple[float, float, float, float, float]]) -> bool:
    if not window:
        return False
    max_phi = max(abs(sample[1]) for sample in window)
    max_phi_dot = max(abs(sample[2]) for sample in window)
    return max_phi < 0.08 and max_phi_dot < 0.35


def main() -> None:
    animations_dir = PROJECT_ROOT / "animations"
    animations_dir.mkdir(exist_ok=True)
    output_path = animations_dir / "frequency_ramp_stabilization_tuned.gif"

    params = PendulumParameters(gravity=9.81, length=1.0, damping=0.3)
    plant = KapitzaPendulumPlant(params)
    simulator = PendulumSimulator(
        plant=plant,
        config=SimulationConfig(
            dt=1.0 / 240.0,
            initial_phi=0.2,
            initial_phi_dot=0.0,
            max_abs_action=140.0,
        ),
    )
    controller = RampHarmonicController(amplitude=97.0, omega_start=18.0, omega_end=21.2, ramp_duration=4.0)

    samples: list[tuple[float, float, float, float, float]] = []
    stabilization_window: list[tuple[float, float, float, float, float]] = []
    frames: list[Image.Image] = []

    frame_interval = 12
    max_steps = int(24.0 / simulator.config.dt)
    stabilized = False

    for step_index in range(max_steps):
        state = simulator.step(controller)
        sample = (
            state.time_value,
            state.phi,
            state.phi_dot,
            state.action,
            controller.current_omega,
        )
        samples.append(sample)
        stabilization_window.append(sample)

        while stabilization_window and state.time_value - stabilization_window[0][0] > 1.5:
            stabilization_window.pop(0)

        if step_index % frame_interval == 0:
            frames.append(
                draw_frame(
                    samples=samples,
                    current_time=state.time_value,
                    current_phi=state.phi,
                    current_action=state.action,
                    current_omega=controller.current_omega,
                    params=params,
                )
            )

        if state.time_value > controller.ramp_duration and has_stabilized(stabilization_window):
            stabilized = True
            break

    if not frames:
        raise RuntimeError("No frames were generated for the GIF animation.")

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000.0 * simulator.config.dt * frame_interval),
        loop=0,
        disposal=2,
    )

    print(f"Saved GIF to: {output_path}")
    if stabilized:
        print(f"Generated {len(frames)} frames up to t = {samples[-1][0]:.2f} s")
    else:
        print(f"Generated {len(frames)} frames over {samples[-1][0]:.2f} s (no stabilization event)")


if __name__ == "__main__":
    main()
