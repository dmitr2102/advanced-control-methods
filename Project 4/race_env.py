from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin, sqrt, tan
import random
from typing import Dict, List, Optional, Tuple


Control = Tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle <= -pi:
        angle += 2.0 * pi
    return angle


def _deg(value: float) -> float:
    return value * pi / 180.0


@dataclass
class CarState:
    x: float
    y: float
    psi: float
    v: float
    delta: float
    torque: float

    def copy(self) -> "CarState":
        return CarState(
            x=self.x,
            y=self.y,
            psi=self.psi,
            v=self.v,
            delta=self.delta,
            torque=self.torque,
        )


@dataclass(frozen=True)
class TrackSegment:
    kind: str
    start_s: float
    length: float
    start_x: float
    start_y: float
    start_psi: float
    end_x: float
    end_y: float
    end_psi: float
    radius: float = 0.0
    signed_angle: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    start_angle: float = 0.0

    @property
    def end_s(self) -> float:
        return self.start_s + self.length

    @property
    def direction(self) -> float:
        if self.signed_angle > 0.0:
            return 1.0
        if self.signed_angle < 0.0:
            return -1.0
        return 0.0


@dataclass(frozen=True)
class TrackProjection:
    s: float
    x: float
    y: float
    psi: float
    lateral: float
    distance: float
    segment_index: int


class SCurveTrack:
    """Fixed S-shaped racing track made from two arcs with different radii."""

    name = "s_curve"
    label = "S curve: R32 left, R54 right"

    def __init__(self) -> None:
        self.width = 12.0
        self.entry_straight = 16.0
        self.first_radius = 32.0
        self.first_angle = _deg(68.0)
        self.transition_straight = 14.0
        self.second_radius = 54.0
        self.second_angle = -_deg(82.0)
        self.exit_straight = 22.0
        self.segments = self._build_segments()
        self.length = self.segments[-1].end_s
        self.min_radius = min(self.first_radius, self.second_radius)
        self.max_curvature = 1.0 / self.min_radius

    def _build_segments(self) -> List[TrackSegment]:
        segments: List[TrackSegment] = []
        x, y, psi, s = 0.0, 0.0, 0.0, 0.0

        def add_straight(length: float) -> None:
            nonlocal x, y, psi, s
            end_x = x + length * cos(psi)
            end_y = y + length * sin(psi)
            segments.append(
                TrackSegment(
                    kind="straight",
                    start_s=s,
                    length=length,
                    start_x=x,
                    start_y=y,
                    start_psi=psi,
                    end_x=end_x,
                    end_y=end_y,
                    end_psi=psi,
                )
            )
            x, y, s = end_x, end_y, s + length

        def add_arc(radius: float, signed_angle: float) -> None:
            nonlocal x, y, psi, s
            direction = 1.0 if signed_angle > 0.0 else -1.0
            left_x, left_y = -sin(psi), cos(psi)
            center_x = x + direction * left_x * radius
            center_y = y + direction * left_y * radius
            start_angle = atan2(y - center_y, x - center_x)
            end_angle = start_angle + signed_angle
            end_x = center_x + radius * cos(end_angle)
            end_y = center_y + radius * sin(end_angle)
            end_psi = wrap_angle(psi + signed_angle)
            length = abs(radius * signed_angle)
            segments.append(
                TrackSegment(
                    kind="arc",
                    start_s=s,
                    length=length,
                    start_x=x,
                    start_y=y,
                    start_psi=psi,
                    end_x=end_x,
                    end_y=end_y,
                    end_psi=end_psi,
                    radius=radius,
                    signed_angle=signed_angle,
                    center_x=center_x,
                    center_y=center_y,
                    start_angle=start_angle,
                )
            )
            x, y, psi, s = end_x, end_y, end_psi, s + length

        add_straight(self.entry_straight)
        add_arc(self.first_radius, self.first_angle)
        add_straight(self.transition_straight)
        add_arc(self.second_radius, self.second_angle)
        add_straight(self.exit_straight)
        return segments

    def centerline_at_s(self, s: float) -> Tuple[float, float]:
        segment, local_s, _ = self._segment_at_s(s)
        if segment.kind == "straight":
            return (
                segment.start_x + local_s * cos(segment.start_psi),
                segment.start_y + local_s * sin(segment.start_psi),
            )

        direction = segment.direction
        theta = segment.start_angle + direction * local_s / segment.radius
        return (
            segment.center_x + segment.radius * cos(theta),
            segment.center_y + segment.radius * sin(theta),
        )

    def tangent_heading_at_s(self, s: float) -> float:
        segment, local_s, _ = self._segment_at_s(s)
        if segment.kind == "straight":
            return segment.start_psi
        return wrap_angle(segment.start_psi + segment.direction * local_s / segment.radius)

    def curvature_at_s(self, s: float) -> float:
        segment, _, _ = self._segment_at_s(s)
        if segment.kind != "arc":
            return 0.0
        return segment.direction / segment.radius

    def progress(self, x: float, y: float) -> float:
        return clamp(self.closest(x, y).s / self.length, 0.0, 1.0)

    def inside(self, x: float, y: float, margin: float = 0.0) -> bool:
        projection = self.closest(x, y)
        return abs(projection.lateral) <= self.width / 2.0 - margin

    def lateral_error(self, x: float, y: float) -> float:
        return self.closest(x, y).lateral

    def heading_error(self, state: CarState) -> float:
        projection = self.closest(state.x, state.y)
        return wrap_angle(projection.psi - state.psi)

    def offset_point(self, s: float, lateral: float) -> Tuple[float, float]:
        x, y = self.centerline_at_s(s)
        psi = self.tangent_heading_at_s(s)
        return x - sin(psi) * lateral, y + cos(psi) * lateral

    def racing_lateral(self, s: float, aggression: float = 1.0) -> float:
        segment, local_s, index = self._segment_at_s(s)
        phase = 0.0 if segment.length <= 0.0 else local_s / segment.length
        amplitude = aggression * min(self.width / 2.0 - 1.35, 4.4)

        if index == 0:
            return _smoothstep(phase) * -amplitude
        if index == 1:
            return _apex_profile(phase, sign=1.0, amplitude=amplitude)
        if index == 2:
            return _lerp(-amplitude, amplitude, _smoothstep(phase))
        if index == 3:
            return _apex_profile(phase, sign=-1.0, amplitude=amplitude)
        return _lerp(amplitude, 0.0, _smoothstep(phase))

    def edge_margin(self, x: float, y: float) -> float:
        return self.width / 2.0 - abs(self.closest(x, y).lateral)

    def closest(self, x: float, y: float) -> TrackProjection:
        best: Optional[TrackProjection] = None
        for index, segment in enumerate(self.segments):
            projection = self._project_to_segment(x, y, segment, index)
            if best is None or projection.distance < best.distance:
                best = projection
        if best is None:
            raise RuntimeError("Track has no segments.")
        return best

    def centerline_points(self, count: int = 240) -> List[Tuple[float, float]]:
        return [self.centerline_at_s(self.length * i / count) for i in range(count + 1)]

    def boundary_points(self, side: float, count: int = 240) -> List[Tuple[float, float]]:
        lateral = side * self.width / 2.0
        return [self.offset_point(self.length * i / count, lateral) for i in range(count + 1)]

    def _segment_at_s(self, s: float) -> Tuple[TrackSegment, float, int]:
        clamped_s = clamp(s, 0.0, self.length)
        for index, segment in enumerate(self.segments):
            if clamped_s <= segment.end_s or index == len(self.segments) - 1:
                return segment, clamped_s - segment.start_s, index
        segment = self.segments[-1]
        return segment, segment.length, len(self.segments) - 1

    def _project_to_segment(
        self,
        x: float,
        y: float,
        segment: TrackSegment,
        index: int,
    ) -> TrackProjection:
        if segment.kind == "straight":
            vx, vy = cos(segment.start_psi), sin(segment.start_psi)
            raw_s = (x - segment.start_x) * vx + (y - segment.start_y) * vy
            local_s = clamp(raw_s, 0.0, segment.length)
            px = segment.start_x + local_s * vx
            py = segment.start_y + local_s * vy
            psi = segment.start_psi
        else:
            angle = atan2(y - segment.center_y, x - segment.center_x)
            along_angle = wrap_angle(angle - segment.start_angle) * segment.direction
            along_angle = clamp(along_angle, 0.0, abs(segment.signed_angle))
            local_s = along_angle * segment.radius
            theta = segment.start_angle + segment.direction * along_angle
            px = segment.center_x + segment.radius * cos(theta)
            py = segment.center_y + segment.radius * sin(theta)
            psi = wrap_angle(segment.start_psi + segment.direction * along_angle)

        left_x, left_y = -sin(psi), cos(psi)
        dx, dy = x - px, y - py
        lateral = dx * left_x + dy * left_y
        return TrackProjection(
            s=segment.start_s + local_s,
            x=px,
            y=py,
            psi=psi,
            lateral=lateral,
            distance=hypot(dx, dy),
            segment_index=index,
        )


def _apex_profile(phase: float, sign: float, amplitude: float) -> float:
    outside = -sign * amplitude
    inside = sign * amplitude
    if phase < 0.5:
        return _lerp(outside, inside, _smoothstep(2.0 * phase))
    return _lerp(inside, outside, _smoothstep(2.0 * phase - 1.0))


def _smoothstep(value: float) -> float:
    x = clamp(value, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _lerp(start: float, end: float, alpha: float) -> float:
    return start + (end - start) * alpha


class RaceCarEnv:
    """Fixed S-curve racetrack with Ackermann kinematics and rear torque actuation."""

    def __init__(self, seed: Optional[int] = None):
        self.track = SCurveTrack()
        self.dt = 0.08
        self.substeps = 4

        self.mass = 1070.0
        self.wheelbase = 2.31
        self.rear_wheel_radius = 0.308
        self.drag_coeff = 0.55
        self.roll_coeff = 42.0

        self.max_steer = 0.62
        self.max_steer_rate = 2.2
        self.steer_tau = 0.18
        self.max_torque_cmd = 1800.0
        self.torque_rate_limit = 6500.0
        self.torque_tau = 0.12
        self.rear_slip_torque = 1450.0
        self.max_lateral_acc = 12.0
        self.max_combined_grip = 1.08
        self.max_speed = 45.0

        self.final_x, self.final_y = self.track.centerline_at_s(self.track.length)
        self.final_psi = self.track.tangent_heading_at_s(self.track.length)
        self.goal_radius = 4.5
        self.goal_heading_tol = 0.42

        self._seed = seed
        self._rng = random.Random(seed)
        self.state = self._initial_state()
        self.time = 0.0
        self.step_count = 0
        self.done = False
        self.won = False
        self.reason = ""

    def reset(self, seed: Optional[int] = None) -> Dict[str, object]:
        if seed is not None:
            self._seed = seed
            self._rng.seed(seed)
        elif self._seed is None:
            self._rng.seed()

        self.state = self._initial_state()
        self.time = 0.0
        self.step_count = 0
        self.done = False
        self.won = False
        self.reason = ""
        return self.observation()

    def observation(self) -> Dict[str, object]:
        return {
            "time": self.time,
            "step_count": self.step_count,
            "state": self.state,
            "done": self.done,
            "won": self.won,
            "reason": self.reason,
        }

    def step(self, control: Control) -> Dict[str, object]:
        if self.done:
            return self.observation()

        torque_cmd, steer_cmd = self.clip_control(control)
        dt = self.dt / self.substeps

        for _ in range(self.substeps):
            self.integrate_state(self.state, torque_cmd, steer_cmd, dt)

            if self.slipping(self.state):
                self._finish(False, self.slip_reason(self.state))
                break
            if self.reached_goal(self.state):
                self._finish(True, "goal")
                break
            if not self.track.inside(self.state.x, self.state.y):
                self._finish(False, "off track")
                break

        self.time += self.dt
        self.step_count += 1
        return self.observation()

    def clip_control(self, control: Control) -> Control:
        torque_cmd, steer_cmd = control
        return (
            clamp(torque_cmd, -self.max_torque_cmd, self.max_torque_cmd),
            clamp(steer_cmd, -self.max_steer, self.max_steer),
        )

    def integrate_state(
        self,
        state: CarState,
        torque_cmd: float,
        steer_cmd: float,
        dt: float,
    ) -> None:
        delta_rate_raw = (steer_cmd - state.delta) / self.steer_tau
        delta_rate = clamp(delta_rate_raw, -self.max_steer_rate, self.max_steer_rate)
        state.delta = clamp(
            state.delta + delta_rate * dt,
            -self.max_steer,
            self.max_steer,
        )

        torque_rate_raw = (torque_cmd - state.torque) / self.torque_tau
        torque_rate = clamp(
            torque_rate_raw,
            -self.torque_rate_limit,
            self.torque_rate_limit,
        )
        state.torque = clamp(
            state.torque + torque_rate * dt,
            -self.max_torque_cmd,
            self.max_torque_cmd,
        )

        traction_force = state.torque / self.rear_wheel_radius
        drag = self.drag_coeff * state.v * abs(state.v)
        rolling = self.roll_coeff * state.v
        v_dot = (traction_force - drag - rolling) / self.mass

        state.v = clamp(state.v + v_dot * dt, 0.0, self.max_speed)
        state.psi = wrap_angle(
            state.psi + state.v / self.wheelbase * tan(state.delta) * dt
        )
        state.x += state.v * cos(state.psi) * dt
        state.y += state.v * sin(state.psi) * dt

    def slipping(self, state: CarState) -> bool:
        return self.grip_usage(state) > self.max_combined_grip

    def slip_reason(self, state: CarState) -> str:
        if abs(state.torque) > self.rear_slip_torque:
            return "rear torque slip"
        if abs(self.lateral_acceleration(state)) > self.max_lateral_acc:
            return "lateral tire slip"
        return "combined tire slip"

    def lateral_acceleration(self, state: CarState) -> float:
        yaw_rate = state.v / self.wheelbase * tan(state.delta)
        return state.v * yaw_rate

    def grip_usage(self, state: CarState) -> float:
        longitudinal = abs(state.torque) / self.rear_slip_torque
        lateral = abs(self.lateral_acceleration(state)) / self.max_lateral_acc
        return sqrt(longitudinal * longitudinal + lateral * lateral)

    def reached_goal(self, state: CarState) -> bool:
        distance = hypot(state.x - self.final_x, state.y - self.final_y)
        heading_error = abs(wrap_angle(self.final_psi - state.psi))
        progress = self.track.progress(state.x, state.y)
        return (
            progress >= 0.985
            and distance <= self.goal_radius
            and heading_error <= self.goal_heading_tol
        )

    def _finish(self, won: bool, reason: str) -> None:
        self.done = True
        self.won = won
        self.reason = reason

    def _initial_state(self) -> CarState:
        x, y = self.track.centerline_at_s(0.0)
        psi = self.track.tangent_heading_at_s(0.0)
        return CarState(
            x=x,
            y=y,
            psi=psi,
            v=11.0,
            delta=0.0,
            torque=250.0,
        )

    def slip_speed_for_torque(self, torque: float) -> float:
        force = torque / self.rear_wheel_radius
        if force <= 0:
            return 0.0
        a = self.drag_coeff
        b = self.roll_coeff
        c = -force
        return max(0.0, (-b + sqrt(b * b - 4 * a * c)) / (2 * a))
