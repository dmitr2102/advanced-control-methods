from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from math import hypot, sqrt
from pathlib import Path
import sys
from time import perf_counter
from typing import Callable, Dict, Iterable, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.controllers.casadi_mpc import solve_casadi_mpc
from src.controllers.sample_mpc import solve_mpc
from src.race_env import Control, RaceCarEnv, wrap_angle


CONFIGS: Tuple[Tuple[int, int], ...] = (
    (35, 60),
    (15, 60),
    (45, 60),
    (35, 20),
    (35, 90),
)


@dataclass(frozen=True)
class RunMetrics:
    controller: str
    horizon: int
    samples: Optional[int]
    won: bool
    reason: str
    steps: int
    track_time_s: float
    finish_speed_kmh: float
    max_speed_kmh: float
    path_length_m: float
    final_distance_m: float
    final_heading_error_rad: float
    max_abs_lateral_m: float
    min_edge_margin_m: float
    max_grip_usage: float
    mean_grip_usage: float
    rms_lateral_acc_mps2: float
    rms_lateral_jerk_mps3: float
    rms_steer_rate_radps: float
    rms_torque_rate_nmps: float
    mean_solve_ms: float
    max_solve_ms: float
    total_solve_s: float
    failed_plans: int


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controllers", default="casadi,sample")
    parser.add_argument(
        "--configs",
        default="35x60,15x60,45x60,35x20,35x90",
        help="Comma-separated horizonxsamples list, for example 35x60,15x60.",
    )
    parser.add_argument("--max-steps", type=int, default=150)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", default="results/mpc_benchmark_metrics.csv")
    args = parser.parse_args()

    controllers = {item.strip().lower() for item in args.controllers.split(",") if item.strip()}
    configs = parse_configs(args.configs)
    rows: List[RunMetrics] = []
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if "casadi" in controllers:
        casadi_cache: Dict[int, RunMetrics] = {}
        for horizon in sorted({horizon for horizon, _ in configs}):
            print(f"running casadi horizon={horizon}", flush=True)
            casadi_cache[horizon] = run_controller(
                controller="casadi",
                horizon=horizon,
                samples=None,
                max_steps=args.max_steps,
                seed=args.seed,
                solver=lambda env, h=horizon: solve_casadi_mpc(env, horizon=h),
            )
            write_csv(output_path, rows + list(casadi_cache.values()))
        for horizon, samples in configs:
            cached = casadi_cache[horizon]
            rows.append(
                RunMetrics(
                    **{
                        **cached.__dict__,
                        "samples": samples,
                    }
                )
            )

    if "sample" in controllers:
        for horizon, samples in configs:
            print(f"running sample horizon={horizon} samples={samples}", flush=True)
            row = run_controller(
                controller="sample",
                horizon=horizon,
                samples=samples,
                max_steps=args.max_steps,
                seed=args.seed,
                solver=lambda env, h=horizon, s=samples: solve_mpc(
                    env,
                    horizon=h,
                    samples=s,
                )
            )
            rows.append(row)
            write_csv(output_path, rows)

    write_csv(output_path, rows)
    print()
    print_table(rows)
    print()
    print(f"saved {output_path}")


def parse_configs(value: str) -> Tuple[Tuple[int, int], ...]:
    result: List[Tuple[int, int]] = []
    for item in value.split(","):
        item = item.strip().lower()
        if not item:
            continue
        horizon_text, samples_text = item.split("x", 1)
        result.append((int(horizon_text), int(samples_text)))
    return tuple(result)


def run_controller(
    controller: str,
    horizon: int,
    samples: Optional[int],
    max_steps: int,
    seed: int,
    solver: Callable[[RaceCarEnv], object],
) -> RunMetrics:
    env = RaceCarEnv(seed=seed)
    positions: List[Tuple[float, float]] = [(env.state.x, env.state.y)]
    controls: List[Control] = []
    solve_times: List[float] = []
    lateral_accs: List[float] = [env.lateral_acceleration(env.state)]
    grip_values: List[float] = [env.grip_usage(env.state)]
    edge_margins: List[float] = [env.track.edge_margin(env.state.x, env.state.y)]
    lateral_values: List[float] = [env.track.closest(env.state.x, env.state.y).lateral]
    speed_values: List[float] = [env.state.v]
    failed_plans = 0

    while not env.done and env.step_count < max_steps:
        started = perf_counter()
        plan = solver(env)
        solve_times.append(perf_counter() - started)
        status = getattr(plan, "status", "")
        if status in {"failed", "tire-slip", "off-track"}:
            failed_plans += 1

        control = getattr(plan, "first_control")
        controls.append(control)
        env.step(control)

        positions.append((env.state.x, env.state.y))
        lateral_accs.append(env.lateral_acceleration(env.state))
        grip_values.append(env.grip_usage(env.state))
        edge_margins.append(env.track.edge_margin(env.state.x, env.state.y))
        lateral_values.append(env.track.closest(env.state.x, env.state.y).lateral)
        speed_values.append(env.state.v)

    path_length = sum(
        hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(positions, positions[1:])
    )
    final_distance = hypot(env.state.x - env.final_x, env.state.y - env.final_y)
    final_heading_error = abs(wrap_angle(env.final_psi - env.state.psi))
    final_speed_kmh = env.state.v * 3.6

    return RunMetrics(
        controller=controller,
        horizon=horizon,
        samples=samples,
        won=env.won,
        reason=env.reason or "max steps" if not env.done else env.reason,
        steps=env.step_count,
        track_time_s=env.step_count * env.dt,
        finish_speed_kmh=final_speed_kmh,
        max_speed_kmh=3.6 * max(speed_values),
        path_length_m=path_length,
        final_distance_m=final_distance,
        final_heading_error_rad=final_heading_error,
        max_abs_lateral_m=max(abs(value) for value in lateral_values),
        min_edge_margin_m=min(edge_margins),
        max_grip_usage=max(grip_values),
        mean_grip_usage=sum(grip_values) / len(grip_values),
        rms_lateral_acc_mps2=rms(lateral_accs),
        rms_lateral_jerk_mps3=rms(differences(lateral_accs, env.dt)),
        rms_steer_rate_radps=rms(control_rate([control[1] for control in controls], env.dt)),
        rms_torque_rate_nmps=rms(control_rate([control[0] for control in controls], env.dt)),
        mean_solve_ms=1000.0 * sum(solve_times) / max(1, len(solve_times)),
        max_solve_ms=1000.0 * max(solve_times, default=0.0),
        total_solve_s=sum(solve_times),
        failed_plans=failed_plans,
    )


def max_speed_from_positions(positions: List[Tuple[float, float]], dt: float) -> float:
    if len(positions) < 2:
        return 0.0
    speeds = [
        hypot(x1 - x0, y1 - y0) / dt
        for (x0, y0), (x1, y1) in zip(positions, positions[1:])
    ]
    return 3.6 * max(speeds)


def differences(values: List[float], dt: float) -> List[float]:
    return [(b - a) / dt for a, b in zip(values, values[1:])]


def control_rate(values: List[float], dt: float) -> List[float]:
    return differences(values, dt)


def rms(values: Iterable[float]) -> float:
    items = list(values)
    if not items:
        return 0.0
    return sqrt(sum(value * value for value in items) / len(items))


def write_csv(path: Path, rows: List[RunMetrics]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(RunMetrics.__dataclass_fields__))
        writer.writeheader()
        for row in rows:
            writer.writerow(row.__dict__)


def print_table(rows: List[RunMetrics]) -> None:
    columns = (
        "controller",
        "horizon",
        "samples",
        "won",
        "steps",
        "track_time_s",
        "finish_speed_kmh",
        "path_length_m",
        "max_abs_lateral_m",
        "rms_lateral_jerk_mps3",
        "rms_steer_rate_radps",
        "mean_solve_ms",
        "failed_plans",
    )
    print(",".join(columns))
    for row in rows:
        values = []
        for column in columns:
            value = getattr(row, column)
            if isinstance(value, float):
                values.append(f"{value:.3f}")
            else:
                values.append(str(value))
        print(",".join(values))


if __name__ == "__main__":
    main()
