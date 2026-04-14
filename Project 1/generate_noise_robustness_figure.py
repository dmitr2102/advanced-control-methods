from __future__ import annotations

import json
import math
import random
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers import CycleEnergyPIDController, LyapunovController  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


DT = 1.0 / 240.0
HORIZON = 15.0
ANGLE_TUBE = 0.10
RATE_TUBE = 0.40
MAX_ABS_ACTION = 140.0
NOISE_LEVELS = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0]
NUM_TRIALS = 40
BASE_SEED = 20260414


@dataclass
class NoiseLevelResult:
    sigma: float
    success_rate: float
    mean_settling_time: float | None
    median_settling_time: float | None
    mean_final_abs_phi: float
    p90_final_abs_phi: float


def rk4_step(plant: KapitzaPendulumPlant, state: tuple[float, float], action: float, dt: float) -> tuple[float, float]:
    def add_scaled(base: tuple[float, float], delta: tuple[float, float], scale: float) -> tuple[float, float]:
        return base[0] + scale * delta[0], base[1] + scale * delta[1]

    k1 = plant.p(state, action)
    k2 = plant.p(add_scaled(state, k1, dt / 2.0), action)
    k3 = plant.p(add_scaled(state, k2, dt / 2.0), action)
    k4 = plant.p(add_scaled(state, k3, dt), action)
    phi = state[0] + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
    phi_dot = state[1] + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
    return phi, phi_dot


def build_lyapunov_controller(params: PendulumParameters) -> LyapunovController:
    return LyapunovController(
        gravity=params.gravity,
        length=params.length,
        carrier_frequency=18.0,
        stiffness_linear=0.35,
        stiffness_cubic=2.35,
        min_amplitude=62.5,
        max_amplitude=104.0,
    )


def build_pid_cycle_controller() -> CycleEnergyPIDController:
    return CycleEnergyPIDController(
        carrier_frequency=14.0,
        base_amplitude=82.0,
        kp=21.8,
        ki=8.5,
        kd=6.6,
        energy_weight=6.3,
        min_amplitude=68.5,
        max_amplitude=133.5,
    )


def simulate_with_noise(controller_factory, sigma: float, trial_seed: int) -> dict[str, float | bool]:
    params = PendulumParameters(damping=0.25)
    plant = KapitzaPendulumPlant(params)
    controller = controller_factory(params) if controller_factory is build_lyapunov_controller else controller_factory()
    if hasattr(controller, "reset"):
        controller.reset()

    rng = random.Random(trial_seed)
    time_value = 0.0
    state = (0.2, 0.0)
    phi_history: list[float] = [state[0]]
    phi_dot_history: list[float] = [state[1]]
    window_len = int(1.5 / DT)

    steps = int(HORIZON / DT)
    for _ in range(steps):
        output = controller.compute_action(time_value, state)
        noisy_action = output.action + rng.gauss(0.0, sigma)
        noisy_action = max(-MAX_ABS_ACTION, min(MAX_ABS_ACTION, noisy_action))
        state = rk4_step(plant, state, noisy_action, DT)
        time_value += DT
        phi_history.append(state[0])
        phi_dot_history.append(state[1])

    convergence_idx = None
    for idx in range(len(phi_history)):
        if all(abs(value) <= ANGLE_TUBE for value in phi_history[idx:]) and all(abs(value) <= RATE_TUBE for value in phi_dot_history[idx:]):
            convergence_idx = idx
            break

    return {
        "success": convergence_idx is not None,
        "settling_time": convergence_idx * DT if convergence_idx is not None else math.nan,
        "final_abs_phi": abs(phi_history[-1]),
    }


def aggregate_results(controller_factory, label: str) -> dict[str, object]:
    level_results: list[NoiseLevelResult] = []
    for level_index, sigma in enumerate(NOISE_LEVELS):
        trial_outcomes = [
            simulate_with_noise(controller_factory, sigma, BASE_SEED + 1000 * level_index + trial_idx)
            for trial_idx in range(NUM_TRIALS)
        ]
        success_times = [float(item["settling_time"]) for item in trial_outcomes if item["success"]]
        final_errors = sorted(float(item["final_abs_phi"]) for item in trial_outcomes)
        success_rate = len(success_times) / NUM_TRIALS
        mean_settling = statistics.fmean(success_times) if success_times else None
        median_settling = statistics.median(success_times) if success_times else None
        p90_index = min(len(final_errors) - 1, math.ceil(0.9 * len(final_errors)) - 1)
        level_results.append(
            NoiseLevelResult(
                sigma=sigma,
                success_rate=success_rate,
                mean_settling_time=mean_settling,
                median_settling_time=median_settling,
                mean_final_abs_phi=statistics.fmean(final_errors),
                p90_final_abs_phi=final_errors[p90_index],
            )
        )
    return {
        "label": label,
        "results": level_results,
    }


def map_x(value: float, left: int, right: int, x_min: float, x_max: float) -> float:
    return left + (value - x_min) / (x_max - x_min) * (right - left)


def map_y(value: float, top: int, bottom: int, y_min: float, y_max: float) -> float:
    return bottom - (value - y_min) / (y_max - y_min) * (bottom - top)


def draw_axes(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    title: str,
    x_label: str,
    y_label: str,
    x_ticks: list[float],
    y_ticks: list[float],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = rect
    font = ImageFont.load_default()
    draw.rounded_rectangle(rect, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)
    draw.text((x0 + 18, y0 + 12), title, fill=(30, 45, 80), font=font)

    left = x0 + 62
    top = y0 + 42
    right = x1 - 18
    bottom = y1 - 44
    draw.line((left, bottom, right, bottom), fill=(90, 90, 90), width=2)
    draw.line((left, top, left, bottom), fill=(90, 90, 90), width=2)

    for tick in y_ticks:
        y = map_y(tick, top, bottom, y_min, y_max)
        draw.line((left, y, right, y), fill=(228, 228, 228), width=1)
        draw.text((x0 + 8, y - 7), f"{tick:.2f}", fill=(90, 90, 90), font=font)

    for tick in x_ticks:
        x = map_x(tick, left, right, x_min, x_max)
        draw.line((x, top, x, bottom), fill=(228, 228, 228), width=1)
        draw.text((x - 10, bottom + 8), f"{tick:.0f}", fill=(90, 90, 90), font=font)

    draw.text((right - 40, bottom + 24), x_label, fill=(55, 55, 60), font=font)
    draw.text((x0 + 10, top - 18), y_label, fill=(55, 55, 60), font=font)
    return left, top, right, bottom


def draw_series(
    draw: ImageDraw.ImageDraw,
    rect_axes: tuple[int, int, int, int],
    x_values: list[float],
    y_values: list[float],
    color: tuple[int, int, int],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> None:
    left, top, right, bottom = rect_axes
    points = [
        (
            map_x(x, left, right, x_min, x_max),
            map_y(y, top, bottom, y_min, y_max),
        )
        for x, y in zip(x_values, y_values)
    ]
    if len(points) > 1:
        draw.line(points, fill=color, width=3)
    for px, py in points:
        draw.ellipse((px - 3, py - 3, px + 3, py + 3), fill=color)


def build_figure(lyapunov_data: dict[str, object], pid_data: dict[str, object]) -> None:
    width, height = 1500, 920
    image = Image.new("RGB", (width, height), (245, 244, 240))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.text((40, 18), "Robustness to Gaussian control noise", fill=(25, 40, 78), font=font)
    draw.text((40, 38), "Comparison between the tuned Lyapunov CLF controller and the tuned PID cycle-energy controller", fill=(70, 70, 78), font=font)

    left_rect = (50, 90, 760, 530)
    right_rect = (790, 90, 1450, 530)
    info_rect = (50, 570, 1450, 880)

    sigmas = [item.sigma for item in lyapunov_data["results"]]
    success_ticks = [0.0, 0.25, 0.5, 0.75, 1.0]
    noise_ticks = [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]

    left_axes = draw_axes(
        draw,
        left_rect,
        "Success rate vs noise level",
        "sigma [m/s^2]",
        "success rate",
        noise_ticks,
        success_ticks,
        0.0,
        20.0,
        0.0,
        1.0,
    )

    settling_y_values = []
    for dataset in (lyapunov_data, pid_data):
        settling_y_values.extend([
            item.median_settling_time if item.median_settling_time is not None else HORIZON
            for item in dataset["results"]
        ])
    settling_max = max(settling_y_values + [HORIZON])
    settling_ticks = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]
    right_axes = draw_axes(
        draw,
        right_rect,
        "Median settling time vs noise level",
        "sigma [m/s^2]",
        "t_settle [s]",
        noise_ticks,
        settling_ticks,
        0.0,
        20.0,
        0.0,
        settling_max,
    )

    palettes = {
        "Lyapunov CLF": (60, 150, 90),
        "PID cycle-energy": (190, 70, 70),
    }
    for dataset in (lyapunov_data, pid_data):
        color = palettes[dataset["label"]]
        x_values = [item.sigma for item in dataset["results"]]
        success_values = [item.success_rate for item in dataset["results"]]
        settling_values = [
            item.median_settling_time if item.median_settling_time is not None else HORIZON
            for item in dataset["results"]
        ]
        draw_series(draw, left_axes, x_values, success_values, color, 0.0, 20.0, 0.0, 1.0)
        draw_series(draw, right_axes, x_values, settling_values, color, 0.0, 20.0, 0.0, settling_max)

    legend_x = 120
    for idx, (label, color) in enumerate(palettes.items()):
        lx = legend_x + idx * 220
        draw.line((lx, 70, lx + 28, 70), fill=color, width=3)
        draw.text((lx + 36, 62), label, fill=(55, 55, 60), font=font)

    draw.rounded_rectangle(info_rect, radius=18, fill=(252, 250, 245), outline=(205, 205, 200), width=2)
    draw.text((68, 588), "Experiment setup", fill=(25, 40, 78), font=font)
    setup_lines = [
        f"Noise model: a_applied(t) = clip(a_cmd(t) + eta(t), +/- {MAX_ABS_ACTION:.0f}), eta ~ N(0, sigma^2)",
        f"Initial condition: phi(0) = 0.2 rad, phi_dot(0) = 0.0 rad/s",
        f"Plant parameters: g = 9.81, l = 1.0, d = 0.25, horizon = {HORIZON:.0f} s, dt = {DT:.5f} s",
        f"Monte Carlo: {NUM_TRIALS} trials for each sigma level",
        "Convergence criterion: the trajectory enters and then stays inside |phi| <= 0.10 rad and |phi_dot| <= 0.40 rad/s",
    ]
    y = 612
    for line in setup_lines:
        draw.text((68, y), line, fill=(50, 50, 55), font=font)
        y += 22

    draw.text((68, 736), "Key observations", fill=(25, 40, 78), font=font)
    lyap_best = lyapunov_data["results"][0]
    pid_best = pid_data["results"][0]
    lyap_last = lyapunov_data["results"][-1]
    pid_last = pid_data["results"][-1]
    obs_lines = [
        f"At sigma = 0, Lyapunov success rate = {100.0 * lyap_best.success_rate:.0f}%, median settling = {lyap_best.median_settling_time:.2f} s.",
        f"At sigma = 0, PID success rate = {100.0 * pid_best.success_rate:.0f}%, median settling = {pid_best.median_settling_time:.2f} s.",
        f"At sigma = {lyap_last.sigma:.0f}, Lyapunov success rate = {100.0 * lyap_last.success_rate:.0f}%, final |phi| p90 = {lyap_last.p90_final_abs_phi:.3f} rad.",
        f"At sigma = {pid_last.sigma:.0f}, PID success rate = {100.0 * pid_last.success_rate:.0f}%, final |phi| p90 = {pid_last.p90_final_abs_phi:.3f} rad.",
    ]
    y = 760
    for line in obs_lines:
        draw.text((68, y), line, fill=(50, 50, 55), font=font)
        y += 22

    output_path = PROJECT_ROOT / "figures" / "noise_robustness_lyapunov_vs_pid_cycle_energy.png"
    output_path.parent.mkdir(exist_ok=True)
    image.save(output_path)


def serialize(dataset: dict[str, object]) -> dict[str, object]:
    return {
        "label": dataset["label"],
        "results": [
            {
                "sigma": item.sigma,
                "success_rate": item.success_rate,
                "mean_settling_time": item.mean_settling_time,
                "median_settling_time": item.median_settling_time,
                "mean_final_abs_phi": item.mean_final_abs_phi,
                "p90_final_abs_phi": item.p90_final_abs_phi,
            }
            for item in dataset["results"]
        ],
    }


def main() -> None:
    lyapunov_data = aggregate_results(build_lyapunov_controller, "Lyapunov CLF")
    pid_data = aggregate_results(build_pid_cycle_controller, "PID cycle-energy")
    build_figure(lyapunov_data, pid_data)

    json_path = PROJECT_ROOT / "figures" / "noise_robustness_lyapunov_vs_pid_cycle_energy.json"
    json_path.write_text(json.dumps({"lyapunov": serialize(lyapunov_data), "pid_cycle_energy": serialize(pid_data)}, indent=2), encoding="utf-8")

    print(f"Saved figure to: {PROJECT_ROOT / 'figures' / 'noise_robustness_lyapunov_vs_pid_cycle_energy.png'}")
    print(f"Saved raw data to: {json_path}")
    for dataset in (lyapunov_data, pid_data):
        print(dataset["label"])
        for item in dataset["results"]:
            median_str = "nan" if item.median_settling_time is None else f"{item.median_settling_time:.3f}"
            print(
                f"  sigma={item.sigma:>4.1f}: success={100.0 * item.success_rate:>5.1f}% "
                f"median_settle={median_str:>6} p90_final_abs_phi={item.p90_final_abs_phi:.3f}"
            )


if __name__ == "__main__":
    main()
