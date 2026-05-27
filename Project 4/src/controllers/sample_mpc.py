from __future__ import annotations

from dataclasses import dataclass
from math import atan, atan2, cos, hypot, sqrt, tan
import random
from typing import List, Optional, Tuple

from src.race_env import CarState, Control, RaceCarEnv, clamp, wrap_angle


DEFAULT_HORIZON = 24
DEFAULT_SAMPLES = 50
INVALID_COST = 1_000_000.0
FULL_THROTTLE_FRACTION = 0.985


@dataclass(frozen=True)
class MPCPlan:
    controls: Tuple[Control, ...]
    path: Tuple[Tuple[float, float], ...]
    endpoints: Tuple[Tuple[float, float], ...]
    cost: float
    status: str
    reaches_goal: bool
    finish_step: Optional[int]
    terminal_speed: float
    sample_count: int
    horizon: int

    @property
    def first_control(self) -> Control:
        return self.controls[0] if self.controls else (0.0, 0.0)


@dataclass
class _Rollout:
    controls: Tuple[Control, ...]
    path: Tuple[Tuple[float, float], ...]
    cost: float
    status: str
    reaches_goal: bool
    finish_step: Optional[int]
    terminal_speed: float


def solve_mpc(
    env: RaceCarEnv,
    horizon: int = DEFAULT_HORIZON,
    samples: int = DEFAULT_SAMPLES,
    seed: Optional[int] = None,
) -> MPCPlan:
    horizon = max(1, horizon)
    samples = max(1, samples)

    if env.done:
        return MPCPlan(
            controls=(),
            path=((env.state.x, env.state.y),),
            endpoints=((env.state.x, env.state.y),),
            cost=0.0,
            status="finished",
            reaches_goal=env.won,
            finish_step=0 if env.won else None,
            terminal_speed=env.state.v,
            sample_count=0,
            horizon=horizon,
        )

    rng = random.Random(seed if seed is not None else _state_seed(env))
    candidates = _candidate_sequences(env, horizon, samples, rng)
    rollouts = [_rollout(env, controls) for controls in candidates]
    sorted_rollouts = sorted(rollouts, key=_rollout_key)
    best = sorted_rollouts[0]
    for rollout in sorted_rollouts:
        if rollout.controls and _one_step_safe(env, rollout.controls[0]):
            best = rollout
            break

    controls = best.controls
    if controls:
        safe_control = _safety_filtered_control(env, controls[0])
        if safe_control != controls[0]:
            controls = (safe_control,) + controls[1:]
            best = _rollout(env, controls)

    return MPCPlan(
        controls=controls,
        path=best.path,
        endpoints=tuple(rollout.path[-1] for rollout in rollouts if rollout.path),
        cost=best.cost,
        status=best.status,
        reaches_goal=best.reaches_goal,
        finish_step=best.finish_step,
        terminal_speed=best.terminal_speed,
        sample_count=len(candidates),
        horizon=horizon,
    )


def _candidate_sequences(
    env: RaceCarEnv,
    horizon: int,
    samples: int,
    rng: random.Random,
) -> List[Tuple[Control, ...]]:
    sequences: List[Tuple[Control, ...]] = []
    speed_low, speed_high = _target_speed_range(env)
    speed_mid = 0.5 * (speed_low + speed_high)
    speed_targets = (
        speed_low,
        speed_mid,
        speed_high,
        min(env.max_speed, 1.15 * speed_high),
    )

    for target_speed in speed_targets:
        for steer_gain in (0.85, 1.15, 1.55, 2.05):
            sequences.append(
                _feedback_sequence(
                    env,
                    horizon,
                    target_speed=target_speed,
                    steer_gain=steer_gain,
                    line_aggression=0.0,
                    full_throttle_progress=0.72,
                )
            )
            sequences.append(
                _feedback_sequence(
                    env,
                    horizon,
                    target_speed=target_speed,
                    steer_gain=steer_gain,
                    line_aggression=0.95,
                    full_throttle_progress=0.60,
                )
            )

    for throttle_progress in (0.50, 0.58, 0.66, 0.74):
        sequences.append(
            _feedback_sequence(
                env,
                horizon,
                target_speed=min(env.max_speed, 1.25 * speed_high),
                steer_gain=1.75,
                line_aggression=1.0,
                full_throttle_progress=throttle_progress,
            )
        )

    projection = env.track.closest(env.state.x, env.state.y)
    nominal_curvature = env.track.curvature_at_s(projection.s + 6.0)
    nominal_steer = atan(env.wheelbase * nominal_curvature)
    torque_levels = (
        0.45 * env.rear_slip_torque,
        0.72 * env.rear_slip_torque,
        FULL_THROTTLE_FRACTION * env.rear_slip_torque,
    )
    for torque in torque_levels:
        for steer_scale in (0.65, 1.0, 1.35):
            sequences.append(tuple((torque, steer_scale * nominal_steer) for _ in range(horizon)))

    while len(sequences) < samples:
        target_speed = rng.uniform(speed_mid, min(env.max_speed, 1.22 * speed_high))
        sequences.append(
            _feedback_sequence(
                env,
                horizon,
                target_speed=target_speed,
                steer_gain=rng.uniform(0.75, 2.4),
                torque_noise=rng.uniform(-50.0, 120.0),
                steer_noise=rng.uniform(-0.08, 0.08),
                line_aggression=rng.uniform(0.35, 1.0),
                full_throttle_progress=rng.uniform(0.52, 0.76),
                rng=rng,
            )
        )

    return sequences[:samples]


def _target_speed_range(env: RaceCarEnv) -> Tuple[float, float]:
    lateral_limited = (6.2 * env.track.min_radius) ** 0.5
    low = clamp(0.62 * lateral_limited, 8.0, 16.0)
    high = clamp(1.20 * lateral_limited, 15.0, min(0.88 * env.max_speed, 32.0))
    if high <= low + 2.0:
        high = min(env.max_speed, low + 2.0)
    return low, high


def _feedback_sequence(
    env: RaceCarEnv,
    horizon: int,
    target_speed: float,
    steer_gain: float,
    torque_noise: float = 0.0,
    steer_noise: float = 0.0,
    line_aggression: float = 0.0,
    full_throttle_progress: Optional[float] = None,
    rng: Optional[random.Random] = None,
) -> Tuple[Control, ...]:
    state = env.state.copy()
    controls: List[Control] = []

    for _ in range(horizon):
        projection = env.track.closest(state.x, state.y)
        progress = projection.s / env.track.length
        lookahead = 8.0 + 0.42 * state.v
        target_s = clamp(projection.s + lookahead, 0.0, env.track.length)
        target_lateral = env.track.racing_lateral(target_s, line_aggression)
        target_x, target_y = env.track.offset_point(target_s, target_lateral)
        target_heading = env.track.tangent_heading_at_s(target_s)
        path_heading = atan2(target_y - state.y, target_x - state.x)

        heading_error = wrap_angle(path_heading - state.psi)
        tangent_error = wrap_angle(target_heading - state.psi)
        final_blend = clamp((progress - 0.64) / 0.30, 0.0, 1.0)
        final_heading_error = wrap_angle(env.final_psi - state.psi)
        desired_lateral_now = env.track.racing_lateral(projection.s, line_aggression)
        lateral_error = projection.lateral - desired_lateral_now

        curvature = env.track.curvature_at_s(target_s)
        steer_ff = atan(env.wheelbase * curvature)
        steer_cmd = (
            steer_ff
            + steer_gain * heading_error
            + 0.35 * tangent_error
            + 2.10 * final_blend * final_heading_error
            - 0.095 * lateral_error
            + steer_noise
        )

        edge_guard = 1.35
        half_width = env.track.width / 2.0
        if projection.lateral > half_width - edge_guard:
            guard = (projection.lateral - (half_width - edge_guard)) / edge_guard
            steer_cmd -= 0.20 + 0.35 * guard
        if projection.lateral < -half_width + edge_guard:
            guard = ((-half_width + edge_guard) - projection.lateral) / edge_guard
            steer_cmd += 0.20 + 0.35 * guard
        if rng is not None:
            steer_cmd += rng.uniform(-0.035, 0.035)
        steer_cmd = clamp(steer_cmd, -env.max_steer, env.max_steer)

        delta_rate_raw = (steer_cmd - state.delta) / env.steer_tau
        delta_rate = clamp(delta_rate_raw, -env.max_steer_rate, env.max_steer_rate)
        predicted_delta = clamp(
            state.delta + delta_rate * env.dt,
            -env.max_steer,
            env.max_steer,
        )
        max_race_torque = _traction_limited_torque(env, state, predicted_delta)
        speed_error = target_speed - state.v
        base_torque = 260.0 + 145.0 * speed_error + torque_noise
        torque_cmd = clamp(base_torque, -0.18 * env.rear_slip_torque, max_race_torque)

        if full_throttle_progress is not None and progress >= full_throttle_progress:
            torque_cmd = max_race_torque
        if progress > 0.88:
            final_error = abs(wrap_angle(env.final_psi - state.psi))
            if final_error > 0.20 or abs(state.delta) > 0.16:
                torque_cmd = min(torque_cmd, 0.45 * env.rear_slip_torque)

        control = env.clip_control((torque_cmd, steer_cmd))
        controls.append(control)
        _integrate_like_env(env, state, control)

    return tuple(controls)


def _rollout(env: RaceCarEnv, controls: Tuple[Control, ...]) -> _Rollout:
    state = env.state.copy()
    path: List[Tuple[float, float]] = [(state.x, state.y)]
    running_cost = 0.0
    control_score = 0.0
    previous_torque_cmd = state.torque
    previous_steer_cmd = state.delta
    previous_lateral = env.track.closest(state.x, state.y).lateral
    previous_lateral_acc = env.lateral_acceleration(state)

    for step_index, control in enumerate(controls, start=1):
        torque_cmd, steer_cmd = env.clip_control(control)
        progress = env.track.progress(state.x, state.y)
        full_torque = FULL_THROTTLE_FRACTION * env.rear_slip_torque
        throttle_fraction = clamp(max(0.0, torque_cmd) / full_torque, 0.0, 1.0)
        exit_weight = 2.7 * clamp((progress - 0.58) / 0.42, 0.0, 1.0)
        control_score += 0.20 * steer_cmd * steer_cmd
        control_score += 0.00002 * min(0.0, torque_cmd) * min(0.0, torque_cmd)
        control_score += _control_smoothness_cost(
            torque_cmd,
            steer_cmd,
            previous_torque_cmd,
            previous_steer_cmd,
        )
        if progress < 0.52:
            control_score += 10.0 * (0.52 - progress) * throttle_fraction
        control_score -= 28.0 * exit_weight * throttle_fraction

        for _ in range(env.substeps):
            _integrate_like_env(env, state, (torque_cmd, steer_cmd), substep=True)
            if env.slipping(state):
                return _invalid_rollout(
                    controls, path, state, "tire-slip", env.track.progress(state.x, state.y)
                )
            if env.reached_goal(state):
                path.append((state.x, state.y))
                return _Rollout(
                    controls=controls,
                    path=tuple(path),
                    cost=_goal_cost(state, step_index, control_score, env),
                    status="goal",
                    reaches_goal=True,
                    finish_step=step_index,
                    terminal_speed=state.v,
                )
            if not env.track.inside(state.x, state.y):
                return _invalid_rollout(
                    controls, path, state, "off-track", env.track.progress(state.x, state.y)
                )

        path.append((state.x, state.y))
        running_cost += _running_cost(env, state)
        running_cost += _track_smoothness_cost(
            env,
            state,
            previous_lateral,
            previous_lateral_acc,
        )
        previous_torque_cmd = torque_cmd
        previous_steer_cmd = steer_cmd
        previous_lateral = env.track.closest(state.x, state.y).lateral
        previous_lateral_acc = env.lateral_acceleration(state)

    return _Rollout(
        controls=controls,
        path=tuple(path),
        cost=_terminal_cost(env, state, running_cost, control_score),
        status="tracking",
        reaches_goal=False,
        finish_step=None,
        terminal_speed=state.v,
    )


def _integrate_like_env(
    env: RaceCarEnv,
    state: CarState,
    control: Control,
    substep: bool = False,
) -> None:
    torque_cmd, steer_cmd = control
    dt = env.dt / env.substeps if substep else env.dt
    env.integrate_state(state, torque_cmd, steer_cmd, dt)


def _invalid_rollout(
    controls: Tuple[Control, ...],
    path: List[Tuple[float, float]],
    state: CarState,
    status: str,
    progress: float,
) -> _Rollout:
    path.append((state.x, state.y))
    return _Rollout(
        controls=controls,
        path=tuple(path),
        cost=INVALID_COST + 10_000.0 * (1.0 - progress),
        status=status,
        reaches_goal=False,
        finish_step=None,
        terminal_speed=state.v,
    )


def _goal_cost(state: CarState, step_index: int, control_score: float, env: RaceCarEnv) -> float:
    heading_error = abs(wrap_angle(env.final_psi - state.psi))
    distance = hypot(state.x - env.final_x, state.y - env.final_y)
    lateral_acc = abs(env.lateral_acceleration(state))
    grip_usage = env.grip_usage(state)
    terminal_stability = (
        1700.0 * state.delta * state.delta
        + 90.0 * lateral_acc * lateral_acc
        + 2600.0 * max(0.0, grip_usage - 0.82) ** 2
    )
    return (
        18.0 * step_index
        + 780.0 * distance
        + 1200.0 * heading_error
        + control_score
        + terminal_stability
        - 520.0 * state.v
    )


def _terminal_cost(
    env: RaceCarEnv,
    state: CarState,
    running_cost: float,
    control_score: float,
) -> float:
    distance = hypot(state.x - env.final_x, state.y - env.final_y)
    heading_error = abs(wrap_angle(env.final_psi - state.psi))
    progress = env.track.progress(state.x, state.y)
    edge_penalty = _edge_penalty(env, state)
    racing_error = 0.0 if progress > 0.92 else _racing_line_error(env, state)
    slip_margin = env.rear_slip_torque - abs(state.torque)
    slip_penalty = 0.0 if slip_margin > 45.0 else 80.0 * (45.0 - slip_margin)
    grip_penalty = 2200.0 * max(0.0, env.grip_usage(state) - 0.82) ** 2
    return (
        5200.0 * (1.0 - progress)
        + 860.0 * distance
        + 1500.0 * heading_error
        + 320.0 * racing_error
        + 1200.0 * edge_penalty
        + 0.18 * running_cost
        + control_score
        + slip_penalty
        + grip_penalty
        - 250.0 * state.v
    )


def _running_cost(env: RaceCarEnv, state: CarState) -> float:
    projection = env.track.closest(state.x, state.y)
    progress = projection.s / env.track.length
    heading_error = abs(env.track.heading_error(state))
    racing_error = _racing_line_error(env, state)
    edge_penalty = _edge_penalty(env, state)
    grip_penalty = max(0.0, env.grip_usage(state) - 0.82) ** 2
    tangent_speed = state.v * cos(heading_error)
    return (
        135.0 * (1.0 - progress)
        + 30.0 * heading_error
        + 42.0 * racing_error
        + 1100.0 * edge_penalty
        + 1800.0 * grip_penalty
        - 20.0 * tangent_speed
        - 9.0 * progress * state.v
    )


def _racing_line_error(env: RaceCarEnv, state: CarState) -> float:
    projection = env.track.closest(state.x, state.y)
    desired = env.track.racing_lateral(projection.s, aggression=1.0)
    return abs(projection.lateral - desired)


def _edge_penalty(env: RaceCarEnv, state: CarState) -> float:
    margin = env.track.edge_margin(state.x, state.y)
    clearance = 1.10
    if margin >= clearance:
        return 0.0
    return (clearance - margin) * (clearance - margin)


def _control_smoothness_cost(
    torque_cmd: float,
    steer_cmd: float,
    previous_torque_cmd: float,
    previous_steer_cmd: float,
) -> float:
    torque_delta = (torque_cmd - previous_torque_cmd) / 1000.0
    steer_delta = steer_cmd - previous_steer_cmd
    return 48.0 * torque_delta * torque_delta + 1250.0 * steer_delta * steer_delta


def _track_smoothness_cost(
    env: RaceCarEnv,
    state: CarState,
    previous_lateral: float,
    previous_lateral_acc: float,
) -> float:
    lateral = env.track.closest(state.x, state.y).lateral
    lateral_delta = lateral - previous_lateral
    lateral_acc_delta = env.lateral_acceleration(state) - previous_lateral_acc
    return 72.0 * lateral_delta * lateral_delta + 18.0 * lateral_acc_delta * lateral_acc_delta


def _traction_limited_torque(
    env: RaceCarEnv,
    state: CarState,
    predicted_delta: Optional[float] = None,
) -> float:
    delta = state.delta if predicted_delta is None else predicted_delta
    lateral_acc = abs(state.v * state.v / env.wheelbase * tan(delta))
    lateral_fraction = clamp(lateral_acc / env.max_lateral_acc, 0.0, 1.0)
    available_fraction = sqrt(max(0.0, 1.0 - lateral_fraction * lateral_fraction))
    return 0.94 * FULL_THROTTLE_FRACTION * env.rear_slip_torque * available_fraction


def _rollout_key(rollout: _Rollout) -> Tuple[int, float, float]:
    return (
        0 if rollout.reaches_goal else 1,
        rollout.cost,
        -rollout.terminal_speed,
    )


def _one_step_safe(env: RaceCarEnv, control: Control) -> bool:
    state = env.state.copy()
    torque_cmd, steer_cmd = env.clip_control(control)
    dt = env.dt / env.substeps
    for _ in range(env.substeps):
        env.integrate_state(state, torque_cmd, steer_cmd, dt)
        if env.slipping(state) or not env.track.inside(state.x, state.y):
            return False
    return True


def _safety_filtered_control(env: RaceCarEnv, control: Control) -> Control:
    torque_cmd, steer_cmd = env.clip_control(control)
    if _one_step_safe(env, (torque_cmd, steer_cmd)):
        return torque_cmd, steer_cmd

    candidates: List[Control] = []
    for steer_scale in (0.75, 0.50, 0.25, 0.0):
        softened_steer = env.state.delta + steer_scale * (steer_cmd - env.state.delta)
        for torque_scale in (0.70, 0.40, 0.15, 0.0):
            softened_torque = max(0.0, torque_cmd) * torque_scale
            candidates.append((softened_torque, softened_steer))
        candidates.append((-0.10 * env.rear_slip_torque, softened_steer))

    safe_candidates = [candidate for candidate in candidates if _one_step_safe(env, candidate)]
    if not safe_candidates:
        return 0.0, env.state.delta

    def score(candidate: Control) -> float:
        state = env.state.copy()
        torque, steer = candidate
        for _ in range(env.substeps):
            env.integrate_state(state, torque, steer, env.dt / env.substeps)
        progress = env.track.progress(state.x, state.y)
        heading_error = abs(env.track.heading_error(state))
        return -progress + 0.05 * heading_error + 0.02 * abs(steer - steer_cmd)

    return min(safe_candidates, key=score)


def _state_seed(env: RaceCarEnv) -> int:
    state = env.state
    value = 41
    value = value * 31 + env.step_count
    value = value * 31 + int(state.x * 1000)
    value = value * 31 + int(state.y * 1000)
    value = value * 31 + int(state.psi * 1000)
    value = value * 31 + int(state.v * 1000)
    value = value * 31 + int(state.delta * 1000)
    value = value * 31 + int(state.torque * 1000)
    return value & 0xFFFFFFFF
