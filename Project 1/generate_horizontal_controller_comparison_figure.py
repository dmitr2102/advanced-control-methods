from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from horizontal_controllers import (  # noqa: E402
    HorizontalAveragedEnergyController,
    HorizontalCycleEnergyPIDController,
    HorizontalHarmonicController,
    HorizontalLyapunovController,
    wrap_to_pi,
)
from horizontal_simulation import HorizontalPendulumSimulator, HorizontalSimulationConfig  # noqa: E402
from horizontal_excitation import HorizontalExcitationParameters  # noqa: E402


ANGLE_TUBE = 0.08
HORIZON = 15.0


@dataclass
class MethodResult:
    label: str
    color: tuple[int, int, int]
    times: list[float]
    angle_error: list[float]
    convergence_time: float | None
    convergence_error: float | None
    tube_percent: float | None


def make_methods(params: HorizontalExcitationParameters) -> list[tuple[str, object, tuple[int, int, int]]]:
    return [
        (
            "Horizontal harmonic",
            HorizontalHarmonicController(gravity=params.gravity, length=params.length),
            (38, 98, 170),
        ),
        (
            "Horizontal Lyapunov",
            HorizontalLyapunovController(gravity=params.gravity, length=params.length),
            (60, 150, 90),
        ),
        (
            "Horizontal averaged-energy",
            HorizontalAveragedEnergyController(gravity=params.gravity, length=params.length),
            (212, 100, 36),
        ),
        (
            "Horizontal PID cycle-energy",
            HorizontalCycleEnergyPIDController(gravity=params.gravity, length=params.length),
            (190, 70, 70),
        ),
    ]


def simulate_method(label: str, controller: object, color: tuple[int, int, int], params: HorizontalExcitationParameters) -> MethodResult:
    target_angle = controller.target_angle
    simulator = HorizontalPendulumSimulator(
        params,
        HorizontalSimulationConfig(
            dt=1.0 / 240.0,
            initial_theta=0.8 * target_angle,
            initial_theta_dot=0.0,
            max_abs_action=1200.0,
        ),
    )
    if hasattr(controller, "reset"):
        controller.reset()

    times = [0.0]
    errors = [abs(wrap_to_pi(simulator.state.theta - target_angle))]

    steps = int(HORIZON / simulator.config.dt)
    for _ in range(steps):
        state = simulator.step(controller)
        times.append(state.time_value)
        errors.append(abs(wrap_to_pi(state.theta - target_angle)))

    convergence_idx = None
    for idx in range(len(errors)):
        if all(value <= ANGLE_TUBE for value in errors[idx:]):
            convergence_idx = idx
            break

    if convergence_idx is None:
        convergence_time = None
        convergence_error = None
        tube_percent = None
    else:
        convergence_time = times[convergence_idx]
        convergence_error = errors[convergence_idx]
        tube_percent = 100.0 * convergence_error / ANGLE_TUBE

    return MethodResult(
        label=label,
        color=color,
        times=times,
        angle_error=errors,
        convergence_time=convergence_time,
        convergence_error=convergence_error,
        tube_percent=tube_percent,
    )


def map_x(t: float, left: int, right: int) -> float:
    return left + (t / HORIZON) * (right - left)


def map_y(v: float, top: int, bottom: int, y_max: float) -> float:
    return bottom - (v / y_max) * (bottom - top)


def draw_series(
    draw: ImageDraw.ImageDraw,
    result: MethodResult,
    left: int,
    top: int,
    right: int,
    bottom: int,
    y_max: float,
) -> None:
    points = [
        (map_x(t, left, right), map_y(v, top, bottom, y_max))
        for t, v in zip(result.times, result.angle_error)
    ]
    if len(points) > 1:
        draw.line(points, fill=result.color, width=3)

    if result.convergence_time is not None and result.convergence_error is not None:
        cx = map_x(result.convergence_time, left, right)
        cy = map_y(result.convergence_error, top, bottom, y_max)
        draw.ellipse((cx - 5, cy - 5, cx + 5, cy + 5), fill=result.color, outline=(255, 255, 255), width=1)
        draw.line((cx, top, cx, bottom), fill=result.color, width=1)


def main() -> None:
    params = HorizontalExcitationParameters(damping=0.5)
    results = [simulate_method(label, controller, color, params) for label, controller, color in make_methods(params)]
    target_angle_deg = math.degrees(HorizontalHarmonicController(gravity=params.gravity, length=params.length).target_angle)

    width, height = 1600, 940
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    title = "Horizontal excitation: angle-error comparison for tuned controllers"
    subtitle = f"Error is measured relative to the averaged side equilibrium theta* = {target_angle_deg:.2f} deg over 15 s."
    draw.text((40, 18), title, fill=(25, 40, 78), font=font)
    draw.text((40, 38), subtitle, fill=(70, 70, 78), font=font)

    plot_rect = (70, 90, 1090, 880)
    info_rect = (1130, 90, 1550, 880)
    x0, y0, x1, y1 = plot_rect
    draw.rounded_rectangle(plot_rect, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)
    draw.rounded_rectangle(info_rect, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)

    left = x0 + 70
    top = y0 + 40
    right = x1 - 22
    bottom = y1 - 55
    y_max = max(0.35, max((max(result.angle_error) for result in results), default=0.35)) * 1.08

    draw.line((left, bottom, right, bottom), fill=(90, 90, 90), width=2)
    draw.line((left, top, left, bottom), fill=(90, 90, 90), width=2)

    for idx in range(6):
        t_value = HORIZON * idx / 5.0
        x = map_x(t_value, left, right)
        draw.line((x, top, x, bottom), fill=(228, 228, 228), width=1)
        draw.text((x - 10, bottom + 10), f"{t_value:.0f}", fill=(85, 85, 90), font=font)

    for idx in range(6):
        value = y_max * idx / 5.0
        y = map_y(value, top, bottom, y_max)
        draw.line((left, y, right, y), fill=(228, 228, 228), width=1)
        draw.text((x0 + 10, y - 7), f"{value:.2f}", fill=(85, 85, 90), font=font)

    tube_y = map_y(ANGLE_TUBE, top, bottom, y_max)
    draw.line((left, tube_y, right, tube_y), fill=(140, 140, 140), width=2)
    draw.text((right - 168, tube_y - 18), "tube: |theta - theta*| = 0.08 rad", fill=(90, 90, 90), font=font)

    for result in results:
        draw_series(draw, result, left, top, right, bottom, y_max)

    draw.text((right - 20, bottom + 28), "t [s]", fill=(55, 55, 60), font=font)
    draw.text((x0 + 10, top - 18), "|theta - theta*| [rad]", fill=(55, 55, 60), font=font)

    legend_x = x0 + 30
    legend_y = y0 + 12
    for idx, result in enumerate(results):
        lx = legend_x + idx * 188
        draw.line((lx, legend_y + 8, lx + 28, legend_y + 8), fill=result.color, width=3)
        draw.text((lx + 36, legend_y), result.label, fill=(55, 55, 60), font=font)

    info_x0, info_y0, info_x1, info_y1 = info_rect
    draw.text((info_x0 + 18, info_y0 + 16), "Controller summary", fill=(25, 40, 78), font=font)
    info_lines = [
        "Reference side equilibrium:",
        f"theta* = {target_angle_deg:.2f} deg",
        "",
        "Carrier settings:",
        "omega = 50 rad/s",
        "reference amplitude = 800 m/s^2",
        "damping = 0.5",
        "",
        "Convergence tube:",
        "|theta - theta*| <= 0.08 rad",
        "",
        "Tube % = 100 * error(t_conv) / 0.08",
    ]
    y = info_y0 + 40
    for line in info_lines:
        draw.text((info_x0 + 18, y), line, fill=(50, 50, 55), font=font)
        y += 20

    y += 12
    for result in results:
        draw.line((info_x0 + 18, y + 8, info_x0 + 42, y + 8), fill=result.color, width=3)
        draw.text((info_x0 + 52, y), result.label, fill=(40, 40, 45), font=font)
        y += 22
        if result.convergence_time is None or result.tube_percent is None:
            draw.text((info_x0 + 52, y), "No convergence by 15 s", fill=(100, 45, 45), font=font)
            y += 30
        else:
            draw.text((info_x0 + 52, y), f"t_conv = {result.convergence_time:.3f} s", fill=(55, 55, 60), font=font)
            y += 18
            draw.text((info_x0 + 52, y), f"tube usage = {result.tube_percent:.1f}%", fill=(55, 55, 60), font=font)
            y += 28

    output_path = PROJECT_ROOT / "figures" / "horizontal_tuned_controller_angle_error_comparison_15s.png"
    output_path.parent.mkdir(exist_ok=True)
    image.save(output_path)

    json_path = PROJECT_ROOT / "figures" / "horizontal_tuned_controller_angle_error_comparison_15s.json"
    json_path.write_text(
        json.dumps(
            {
                "tube_radius": ANGLE_TUBE,
                "target_angle_deg": target_angle_deg,
                "results": [
                    {
                        "label": result.label,
                        "convergence_time": result.convergence_time,
                        "tube_percent": result.tube_percent,
                    }
                    for result in results
                ],
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"Saved comparison figure to: {output_path}")
    print(f"Saved summary data to: {json_path}")
    for result in results:
        if result.convergence_time is None or result.tube_percent is None:
            print(f"{result.label}: no convergence by {HORIZON:.1f} s")
        else:
            print(
                f"{result.label}: t_conv = {result.convergence_time:.3f} s, "
                f"tube usage = {result.tube_percent:.1f}%"
            )


if __name__ == "__main__":
    main()
