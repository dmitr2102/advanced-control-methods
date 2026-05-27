from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from math import hypot, sqrt
from pathlib import Path
import sys
from time import perf_counter
from typing import Iterable, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT / "src"
for path in (SRC_ROOT,):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from controllers.obstacle_mpc import ObstacleCasadiMPCController
from obstacle_env import ObstacleRaceEnv
from race_env import Control, wrap_angle


@dataclass(frozen=True)
class ObstacleMetrics:
    layout: str
    horizon: int
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
    min_obstacle_margin_m: float
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--layouts", default="inside,second_outside")
    parser.add_argument("-n", "--horizon", type=int, default=15)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=170)
    parser.add_argument("--output", default="results/obstacle_mpc_metrics.csv")
    args = parser.parse_args()

    layouts = [item.strip() for item in args.layouts.split(",") if item.strip()]
    rows: List[ObstacleMetrics] = []
    for layout in layouts:
        print(f"running obstacle layout={layout} horizon={args.horizon}", flush=True)
        row = run_layout(layout, args.horizon, args.seed, args.max_steps)
        rows.append(row)
        write_csv(Path(args.output), rows)

    print_table(rows)
    print(f"saved {args.output}")
    return 0


def run_layout(layout: str, horizon: int, seed: int, max_steps: int) -> ObstacleMetrics:
    env = ObstacleRaceEnv(seed=seed, layout=layout)
    controller = ObstacleCasadiMPCController(horizon=horizon)
    positions: List[Tuple[float, float]] = [(env.state.x, env.state.y)]
    controls: List[Control] = []
    solve_times: List[float] = []
    speed_values: List[float] = [env.state.v]
    lateral_accs: List[float] = [env.lateral_acceleration(env.state)]
    grip_values: List[float] = [env.grip_usage(env.state)]
    edge_margins: List[float] = [env.track.edge_margin(env.state.x, env.state.y)]
    obstacle_margins: List[float] = [obstacle_margin(env)]
    lateral_values: List[float] = [env.track.closest(env.state.x, env.state.y).lateral]
    failed_plans = 0

    while not env.done and env.step_count < max_steps:
        started = perf_counter()
        plan = controller.solve(env)
        solve_times.append(perf_counter() - started)
        if plan.status == "failed":
            failed_plans += 1
        control = plan.first_control
        controls.append(control)
        env.step(control)

        positions.append((env.state.x, env.state.y))
        speed_values.append(env.state.v)
        lateral_accs.append(env.lateral_acceleration(env.state))
        grip_values.append(env.grip_usage(env.state))
        edge_margins.append(env.track.edge_margin(env.state.x, env.state.y))
        obstacle_margins.append(obstacle_margin(env))
        lateral_values.append(env.track.closest(env.state.x, env.state.y).lateral)

    path_length = sum(
        hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(positions, positions[1:])
    )
    final_distance = hypot(env.state.x - env.final_x, env.state.y - env.final_y)
    final_heading_error = abs(wrap_angle(env.final_psi - env.state.psi))
    return ObstacleMetrics(
        layout=layout,
        horizon=horizon,
        won=env.won,
        reason=env.reason or ("max steps" if not env.done else env.reason),
        steps=env.step_count,
        track_time_s=env.step_count * env.dt,
        finish_speed_kmh=env.state.v * 3.6,
        max_speed_kmh=max(speed_values) * 3.6,
        path_length_m=path_length,
        final_distance_m=final_distance,
        final_heading_error_rad=final_heading_error,
        max_abs_lateral_m=max(abs(value) for value in lateral_values),
        min_edge_margin_m=min(edge_margins),
        min_obstacle_margin_m=min(obstacle_margins),
        max_grip_usage=max(grip_values),
        mean_grip_usage=sum(grip_values) / len(grip_values),
        rms_lateral_acc_mps2=rms(lateral_accs),
        rms_lateral_jerk_mps3=rms(differences(lateral_accs, env.dt)),
        rms_steer_rate_radps=rms(differences([control[1] for control in controls], env.dt)),
        rms_torque_rate_nmps=rms(differences([control[0] for control in controls], env.dt)),
        mean_solve_ms=1000.0 * sum(solve_times) / max(1, len(solve_times)),
        max_solve_ms=1000.0 * max(solve_times, default=0.0),
        total_solve_s=sum(solve_times),
        failed_plans=failed_plans,
    )


def obstacle_margin(env: ObstacleRaceEnv) -> float:
    projection = env.track.closest(env.state.x, env.state.y)
    margins = []
    for obstacle in env.obstacles:
        ds = max(0.0, abs(projection.s - obstacle.s) - obstacle.half_s)
        de = max(0.0, abs(projection.lateral - obstacle.lateral_center) - obstacle.half_lateral)
        margins.append(sqrt(ds * ds + de * de))
    return min(margins)


def differences(values: List[float], dt: float) -> List[float]:
    return [(b - a) / dt for a, b in zip(values, values[1:])]


def rms(values: Iterable[float]) -> float:
    items = list(values)
    if not items:
        return 0.0
    return sqrt(sum(value * value for value in items) / len(items))


def write_csv(path: Path, rows: List[ObstacleMetrics]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(ObstacleMetrics.__dataclass_fields__))
        writer.writeheader()
        for row in rows:
            writer.writerow(row.__dict__)


def print_table(rows: List[ObstacleMetrics]) -> None:
    print("layout,horizon,won,steps,time_s,exit_kmh,path_m,min_obs_margin,jerk,steer_rate,mean_solve_ms")
    for row in rows:
        print(
            f"{row.layout},{row.horizon},{row.won},{row.steps},"
            f"{row.track_time_s:.3f},{row.finish_speed_kmh:.3f},"
            f"{row.path_length_m:.3f},{row.min_obstacle_margin_m:.3f},"
            f"{row.rms_lateral_jerk_mps3:.3f},{row.rms_steer_rate_radps:.3f},"
            f"{row.mean_solve_ms:.3f}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
