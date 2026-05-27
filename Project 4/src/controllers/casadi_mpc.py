from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin, sqrt
from typing import List, Optional, Tuple

from src.race_env import CarState, Control, RaceCarEnv, clamp, wrap_angle


@dataclass(frozen=True)
class CasadiMPCPlan:
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


class CasadiMPCController:
    """Nonlinear MPC solved by CasADi/IPOPT.

    The optimization uses the same six-state plant as RaceCarEnv:

        s = [x, y, psi, v, delta, T]
        a = [T_cmd, steer_cmd]

    Track limits are imposed as a tube around fixed look-ahead reference points
    on the S-curve. This keeps the NLP compact while still enforcing that the
    car remains inside the road corridor.
    """

    def __init__(
        self,
        horizon: int = 24,
        track_margin: float = 1.20,
        reference_aggression: float = 0.85,
        ipopt_max_iter: int = 220,
    ) -> None:
        self.horizon = max(2, horizon)
        self.track_margin = track_margin
        self.reference_aggression = reference_aggression
        self.ipopt_max_iter = ipopt_max_iter

    def solve(self, env: RaceCarEnv) -> CasadiMPCPlan:
        try:
            import casadi as ca
        except ImportError as exc:
            raise RuntimeError(
                "CasADi is not installed for this Python interpreter. "
                "Install dependencies with: python -m pip install -r requirements.txt"
            ) from exc

        if env.done:
            return CasadiMPCPlan(
                controls=(),
                path=((env.state.x, env.state.y),),
                endpoints=((env.state.x, env.state.y),),
                cost=0.0,
                status="finished",
                reaches_goal=env.won,
                finish_step=0 if env.won else None,
                terminal_speed=env.state.v,
                sample_count=1,
                horizon=self.horizon,
            )

        n = self.horizon
        dt = env.dt
        opti = ca.Opti()
        x = opti.variable(6, n + 1)
        u = opti.variable(2, n)
        finish_slack = opti.variable()

        x0 = _frenet_state_vector(env)
        opti.subject_to(x[:, 0] == x0)
        opti.subject_to(finish_slack >= 0.0)

        half_width = env.track.width / 2.0 - self.track_margin
        track_length = env.track.length
        max_predicted_s = track_length + 80.0

        cost = 0.0
        previous_torque_cmd = env.state.torque
        previous_steer_cmd = env.state.delta

        for k in range(n):
            xk = x[:, k]
            uk = u[:, k]
            x_next = _casadi_frenet_step(ca, env, xk, uk, dt)
            opti.subject_to(x[:, k + 1] == x_next)

            torque_rate = (uk[0] - xk[5]) / env.torque_tau
            steer_rate = (uk[1] - xk[4]) / env.steer_tau
            opti.subject_to(opti.bounded(-env.torque_rate_limit, torque_rate, env.torque_rate_limit))
            opti.subject_to(opti.bounded(-env.max_steer_rate, steer_rate, env.max_steer_rate))

            lateral_acc = xk[3] * xk[3] / env.wheelbase * ca.tan(xk[4])
            grip_usage = (xk[5] / env.rear_slip_torque) ** 2 + (
                lateral_acc / env.max_lateral_acc
            ) ** 2

            opti.subject_to(opti.bounded(0.0, xk[0], max_predicted_s))
            opti.subject_to(opti.bounded(-half_width, xk[1], half_width))
            opti.subject_to(opti.bounded(-1.20, xk[2], 1.20))
            opti.subject_to(grip_usage <= env.max_combined_grip * env.max_combined_grip)

            torque_delta = (uk[0] - previous_torque_cmd) / 1000.0
            steer_delta = uk[1] - previous_steer_cmd
            progress = xk[0] / track_length
            exit_blend = _blend_ca(ca, progress, 0.62, 0.92)
            goal_blend = _blend_ca(ca, progress, 0.82, 0.985)
            lookahead_s = xk[0] + 18.0 + 0.45 * xk[3]
            raw_lookahead_curvature = _curvature_at_s(ca, env, lookahead_s)
            lookahead_curvature = ca.fmax(1e-4, ca.fabs(raw_lookahead_curvature))
            speed_factor = 0.90 + 0.18 * exit_blend
            curvature_speed = ca.fmin(
                env.max_speed,
                speed_factor * ca.sqrt(env.max_lateral_acc / lookahead_curvature),
            )
            speed_excess = ca.fmax(0.0, xk[3] - curvature_speed)
            dive_direction = ca.if_else(raw_lookahead_curvature >= 0.0, 1.0, -1.0)
            dive_activation = ca.fmin(1.0, lookahead_curvature / env.track.max_curvature)
            dive_target = dive_direction * (0.72 * half_width) * dive_activation
            dive_distance = 12.0 + 0.25 * xk[3]
            dive_heading = ca.atan((dive_target - xk[1]) / dive_distance)
            dive_steer = ca.atan(env.wheelbase * raw_lookahead_curvature)
            edge_softness = half_width - 1.55
            edge_penalty = ca.fmax(0.0, ca.fabs(xk[1]) - edge_softness)
            grip_buffer = ca.fmax(0.0, ca.sqrt(grip_usage) - 0.98)
            finish_gap = ca.fmax(0.0, track_length - xk[0])
            finish_delta = track_length - xk[0]
            finish_gate = ca.exp(-0.5 * (finish_delta / 28.0) ** 2)
            brake_torque = ca.fmin(0.0, uk[0])

            cost += (160.0 + 120.0 * exit_blend) * (finish_gap / track_length)
            cost += (42.0 + 90.0 * exit_blend + 80.0 * goal_blend) * xk[2] * xk[2]
            cost += (120.0 + 45.0 * exit_blend) * dive_activation * (xk[2] - dive_heading) ** 2
            cost += (6.50 + 0.45 * finish_gate) * xk[1] * xk[1]
            cost += (18.0 + 8.0 * exit_blend) * dive_activation * (xk[1] - dive_target) ** 2
            cost += 520.0 * dive_activation * (uk[1] - 1.85 * dive_steer) ** 2
            cost += 140.0 * speed_excess * speed_excess
            cost += 0.000006 * uk[0] * uk[0]
            cost += 0.0140 * brake_torque * brake_torque
            cost += (0.4 + 9.0 * finish_gate) * uk[1] * uk[1]
            cost += 18.0 * torque_delta * torque_delta
            cost += 360.0 * steer_delta * steer_delta
            cost += 4.0 * (lateral_acc / env.max_lateral_acc) ** 2
            cost += 28000.0 * edge_penalty * edge_penalty
            cost += 3600.0 * grip_buffer * grip_buffer
            cost += 180.0 * goal_blend * xk[4] * xk[4]
            cost -= (8.0 + 28.0 * exit_blend + 140.0 * finish_gate) * x_next[3]
            cost -= 0.030 * exit_blend * uk[0]
            cost -= 0.040 * progress * uk[0]

            previous_torque_cmd = uk[0]
            previous_steer_cmd = uk[1]

        terminal = x[:, n]
        terminal_gap = ca.fmax(0.0, track_length - terminal[0])
        terminal_delta = track_length - terminal[0]
        terminal_gate = ca.exp(-0.5 * (terminal_delta / env.goal_radius) ** 2)
        opti.subject_to(opti.bounded(0.0, terminal[0], max_predicted_s))
        opti.subject_to(opti.bounded(-half_width, terminal[1], half_width))
        opti.subject_to(terminal[0] + finish_slack >= track_length - env.goal_radius)
        cost += 16000.0 * (finish_slack / track_length) ** 2
        cost += 9000.0 * (terminal_gap / track_length) ** 2
        cost += terminal_gate * (
            6500.0 * (terminal[1] / env.goal_radius) ** 2
            + 7200.0 * terminal[2] * terminal[2]
            + 16000.0 * terminal[4] * terminal[4]
            + 3400.0 * u[1, n - 1] * u[1, n - 1]
        )
        cost -= (520.0 + 1800.0 * terminal_gate) * terminal[3]
        cost -= 0.180 * terminal_gate * u[0, n - 1]

        opti.minimize(cost)
        opti.subject_to(opti.bounded(0.0, x[3, :], env.max_speed))
        opti.subject_to(opti.bounded(-env.max_steer, x[4, :], env.max_steer))
        opti.subject_to(opti.bounded(-env.rear_slip_torque, x[5, :], env.rear_slip_torque))
        opti.subject_to(opti.bounded(-env.max_torque_cmd, u[0, :], env.max_torque_cmd))
        opti.subject_to(opti.bounded(-env.max_steer, u[1, :], env.max_steer))

        self._set_initial_guess(opti, x, u, finish_slack, env)
        opti.solver(
            "ipopt",
            {"print_time": False},
            {
                "print_level": 0,
                "max_iter": self.ipopt_max_iter,
                "acceptable_tol": 1e-4,
                "acceptable_obj_change_tol": 1e-3,
            },
        )

        status = "optimal"
        try:
            solution = opti.solve()
            x_value = solution.value(x)
            u_value = solution.value(u)
            cost_value = float(solution.value(cost))
        except RuntimeError:
            status = "failed"
            x_value = opti.debug.value(x)
            u_value = opti.debug.value(u)
            cost_value = float("inf")

        controls = tuple((float(u_value[0, k]), float(u_value[1, k])) for k in range(n))
        controls = _filter_unsafe_first_control(env, controls)
        path = _frenet_path(env, x_value)

        finish_step = _first_goal_step_frenet(env, x_value)
        return CasadiMPCPlan(
            controls=controls,
            path=path,
            endpoints=(path[-1],),
            cost=cost_value,
            status=status,
            reaches_goal=finish_step is not None,
            finish_step=finish_step,
            terminal_speed=float(x_value[3, -1]),
            sample_count=1,
            horizon=n,
        )

    def _set_initial_guess(self, opti, x, u, finish_slack, env: RaceCarEnv) -> None:
        projection = env.track.closest(env.state.x, env.state.y)
        lookahead_speed = max(env.state.v, 18.0)
        remaining = max(0.0, env.track.length - projection.s - env.goal_radius)
        opti.set_initial(finish_slack, remaining)
        for k in range(self.horizon + 1):
            s_guess = min(env.track.length, projection.s + lookahead_speed * env.dt * k)
            e_guess = _dive_lateral_guess(env, s_guess, self.track_margin)
            progress = s_guess / env.track.length
            speed_guess = min(env.max_speed, env.state.v + 0.18 * k + 10.0 * progress)
            opti.set_initial(x[0, k], s_guess)
            opti.set_initial(x[1, k], e_guess)
            opti.set_initial(x[2, k], 0.0)
            opti.set_initial(x[3, k], speed_guess)
            opti.set_initial(x[4, k], env.state.delta)
            opti.set_initial(x[5, k], min(0.75 * env.rear_slip_torque, max(0.0, env.state.torque)))
        for k in range(self.horizon):
            opti.set_initial(u[0, k], min(0.80 * env.rear_slip_torque, max(0.0, env.state.torque)))
            s_guess = min(env.track.length, projection.s + lookahead_speed * env.dt * k)
            opti.set_initial(u[1, k], _dive_steer_guess(env, s_guess))


def solve_casadi_mpc(
    env: RaceCarEnv,
    horizon: int = 24,
    track_margin: float = 1.20,
) -> CasadiMPCPlan:
    return CasadiMPCController(horizon=horizon, track_margin=track_margin).solve(env)


def _state_vector(state: CarState) -> List[float]:
    return [state.x, state.y, state.psi, state.v, state.delta, state.torque]


def _frenet_state_vector(env: RaceCarEnv) -> List[float]:
    projection = env.track.closest(env.state.x, env.state.y)
    heading_error = wrap_angle(env.state.psi - projection.psi)
    return [
        projection.s,
        projection.lateral,
        heading_error,
        env.state.v,
        env.state.delta,
        env.state.torque,
    ]


def _dive_lateral_guess(env: RaceCarEnv, s: float, track_margin: float) -> float:
    lookahead_s = min(env.track.length, s + 18.0)
    curvature = env.track.curvature_at_s(lookahead_s)
    if abs(curvature) < 1e-5:
        return 0.0
    half_width = env.track.width / 2.0 - track_margin
    activation = min(1.0, abs(curvature) / env.track.max_curvature)
    direction = 1.0 if curvature > 0.0 else -1.0
    return direction * 0.72 * half_width * activation


def _dive_steer_guess(env: RaceCarEnv, s: float) -> float:
    lookahead_s = min(env.track.length, s + 18.0)
    curvature = env.track.curvature_at_s(lookahead_s)
    return clamp(1.85 * env.wheelbase * curvature, -env.max_steer, env.max_steer)


def _blend_ca(ca, value, start: float, end: float):
    if end <= start:
        return 1.0
    raw = (value - start) / (end - start)
    return ca.fmin(1.0, ca.fmax(0.0, raw))


def _curvature_at_s(ca, env: RaceCarEnv, s):
    curvature = 0.0
    for segment in env.track.segments:
        if segment.kind == "arc":
            value = segment.direction / segment.radius
        else:
            value = 0.0
        curvature = ca.if_else(s >= segment.start_s, value, curvature)
    return curvature


def _casadi_frenet_step(ca, env: RaceCarEnv, state, control, dt: float):
    next_state = state
    sub_dt = dt / env.substeps
    for _ in range(env.substeps):
        next_state = _casadi_frenet_substep(ca, env, next_state, control, sub_dt)
    return next_state


def _casadi_frenet_substep(ca, env: RaceCarEnv, state, control, dt: float):
    s = state[0]
    e = state[1]
    psi_e = state[2]
    v = state[3]
    delta = state[4]
    torque = state[5]

    next_delta = delta + (control[1] - delta) / env.steer_tau * dt
    next_torque = torque + (control[0] - torque) / env.torque_tau * dt

    traction_force = next_torque / env.rear_wheel_radius
    drag = env.drag_coeff * v * v
    rolling = env.roll_coeff * v
    next_v = v + (traction_force - drag - rolling) / env.mass * dt

    curvature = _curvature_at_s(ca, env, s)
    denominator = 1.0 - curvature * e
    s_dot = next_v * ca.cos(psi_e) / denominator
    e_dot = next_v * ca.sin(psi_e)
    yaw_rate = next_v / env.wheelbase * ca.tan(next_delta)
    psi_e_dot = yaw_rate - curvature * s_dot

    return ca.vertcat(
        s + s_dot * dt,
        e + e_dot * dt,
        psi_e + psi_e_dot * dt,
        next_v,
        next_delta,
        next_torque,
    )


def _frenet_path(env: RaceCarEnv, x_value) -> Tuple[Tuple[float, float], ...]:
    return tuple(
        env.track.offset_point(float(x_value[0, k]), float(x_value[1, k]))
        for k in range(x_value.shape[1])
    )


def _first_goal_step_frenet(env: RaceCarEnv, x_value) -> Optional[int]:
    for k in range(1, x_value.shape[1]):
        s = float(x_value[0, k])
        e = float(x_value[1, k])
        psi_e = float(x_value[2, k])
        x, y = env.track.offset_point(s, e)
        psi = wrap_angle(env.track.tangent_heading_at_s(s) + psi_e)
        distance = hypot(x - env.final_x, y - env.final_y)
        heading_error = abs(wrap_angle(env.final_psi - psi))
        progress = clamp(s / env.track.length, 0.0, 1.0)
        if (
            progress >= 0.985
            and distance <= env.goal_radius
            and heading_error <= env.goal_heading_tol
        ):
            return k
    return None


def _reference(
    env: RaceCarEnv,
    horizon: int,
    aggression: float,
) -> List[Tuple[float, float, float, float, float, float, float]]:
    projection = env.track.closest(env.state.x, env.state.y)
    ds = clamp(max(env.state.v, 14.0) * env.dt * 1.25, 1.35, 2.55)
    result: List[Tuple[float, float, float, float, float, float, float]] = []
    for k in range(horizon + 1):
        s = clamp(projection.s + ds * k, 0.0, env.track.length)
        lateral = env.track.racing_lateral(s, aggression=aggression)
        x, y = env.track.centerline_at_s(s)
        psi = env.track.tangent_heading_at_s(s)
        curvature = abs(env.track.curvature_at_s(s))
        if curvature < 1e-5:
            target_speed = min(env.max_speed, 31.0)
        else:
            target_speed = min(env.max_speed, 0.92 * sqrt(env.max_lateral_acc / curvature))
        normal_x = -sin(psi)
        normal_y = cos(psi)
        result.append((x, y, psi, normal_x, normal_y, target_speed, lateral))
    return result


def _casadi_step(ca, env: RaceCarEnv, state, control, dt: float):
    delta_rate_raw = (control[1] - state[4]) / env.steer_tau
    next_delta = state[4] + delta_rate_raw * dt

    torque_rate_raw = (control[0] - state[5]) / env.torque_tau
    next_torque = state[5] + torque_rate_raw * dt

    traction_force = next_torque / env.rear_wheel_radius
    drag = env.drag_coeff * state[3] * state[3]
    rolling = env.roll_coeff * state[3]
    v_dot = (traction_force - drag - rolling) / env.mass
    next_v = state[3] + v_dot * dt
    next_psi = state[2] + next_v / env.wheelbase * ca.tan(next_delta) * dt
    next_x = state[0] + next_v * ca.cos(next_psi) * dt
    next_y = state[1] + next_v * ca.sin(next_psi) * dt
    return ca.vertcat(next_x, next_y, next_psi, next_v, next_delta, next_torque)


def _first_goal_step(env: RaceCarEnv, path, x_value) -> Optional[int]:
    for k, (x, y) in enumerate(path[1:], start=1):
        distance = hypot(x - env.final_x, y - env.final_y)
        heading_error = abs(wrap_angle(env.final_psi - float(x_value[2, k])))
        progress = env.track.progress(x, y)
        if (
            progress >= 0.985
            and distance <= env.goal_radius
            and heading_error <= env.goal_heading_tol
        ):
            return k
    return None


def _filter_unsafe_first_control(
    env: RaceCarEnv,
    controls: Tuple[Control, ...],
) -> Tuple[Control, ...]:
    if not controls:
        return controls
    first = env.clip_control(controls[0])
    if _one_step_safe(env, first):
        return (first,) + controls[1:]

    torque_cmd, steer_cmd = first
    candidates: List[Control] = []
    for steer_scale in (0.75, 0.5, 0.25, 0.0):
        steer = env.state.delta + steer_scale * (steer_cmd - env.state.delta)
        for torque_scale in (0.6, 0.3, 0.0):
            candidates.append((max(0.0, torque_cmd) * torque_scale, steer))
        candidates.append((-0.08 * env.rear_slip_torque, steer))

    safe = [candidate for candidate in candidates if _one_step_safe(env, candidate)]
    if not safe:
        return ((0.0, env.state.delta),) + controls[1:]
    return (max(safe, key=lambda candidate: candidate[0]),) + controls[1:]


def _one_step_safe(env: RaceCarEnv, control: Control) -> bool:
    state = env.state.copy()
    torque_cmd, steer_cmd = env.clip_control(control)
    dt = env.dt / env.substeps
    for _ in range(env.substeps):
        env.integrate_state(state, torque_cmd, steer_cmd, dt)
        if env.slipping(state) or not env.track.inside(state.x, state.y):
            return False
    return True
