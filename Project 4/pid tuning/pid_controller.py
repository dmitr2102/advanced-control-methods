from __future__ import annotations

from dataclasses import dataclass
from math import atan
from pathlib import Path
import sys
from typing import Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from race_env import CarState, Control, RaceCarEnv, clamp, wrap_angle


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


class CenterlinePIDController:
    """PID-style controller for keeping the car near the track centerline."""

    def __init__(
        self,
        env: RaceCarEnv,
        lateral_gains: PIDGains = PIDGains(kp=0.03, ki=0.002, kd=0.015),
        heading_gain: float = 0.3,
        curvature_gain: float = 1.0,
        target_speed: float = 32.0,
        speed_gain: float = 135.0,
    ) -> None:
        self.env = env
        self.lateral_gains = lateral_gains
        self.heading_gain = heading_gain
        self.curvature_gain = curvature_gain
        self.target_speed = target_speed
        self.speed_gain = speed_gain
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_control: Control = (env.state.torque, env.state.delta)

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_control = (self.env.state.torque, self.env.state.delta)

    def control(self, state: CarState | None = None) -> Control:
        state = self.env.state if state is None else state
        projection = self.env.track.closest(state.x, state.y)

        # Positive lateral error means the car is left of the centerline.
        lateral_error = projection.lateral
        self.integral = clamp(
            self.integral + lateral_error * self.env.dt,
            -16.0,
            16.0,
        )
        derivative = (lateral_error - self.previous_error) / self.env.dt
        self.previous_error = lateral_error

        lookahead_s = clamp(
            projection.s + 6.0 + 0.35 * state.v,
            0.0,
            self.env.track.length,
        )
        center_heading = self.env.track.tangent_heading_at_s(lookahead_s)
        heading_error = wrap_angle(center_heading - state.psi)
        curvature = self.env.track.curvature_at_s(lookahead_s)
        feed_forward = atan(self.env.wheelbase * curvature)

        correction = (
            -self.lateral_gains.kp * lateral_error
            - self.lateral_gains.ki * self.integral
            - self.lateral_gains.kd * derivative
            + self.heading_gain * heading_error
        )
        steer_cmd = clamp(
            self.curvature_gain * feed_forward + correction,
            -self.env.max_steer,
            self.env.max_steer,
        )

        speed_error = self.target_speed - state.v
        torque_cmd = 310.0 + self.speed_gain * speed_error

        edge_margin = self.env.track.edge_margin(state.x, state.y)
        if edge_margin < 1.5 or abs(heading_error) > 0.55:
            torque_cmd = min(torque_cmd, 0.35 * self.env.rear_slip_torque)

        lateral_fraction = clamp(
            abs(self.env.lateral_acceleration(state)) / self.env.max_lateral_acc,
            0.0,
            1.0,
        )
        torque_limit = (0.78 - 0.38 * lateral_fraction) * self.env.rear_slip_torque
        torque_cmd = clamp(torque_cmd, -0.18 * self.env.rear_slip_torque, torque_limit)

        self.previous_control = self.env.clip_control((torque_cmd, steer_cmd))
        return self.previous_control


def solve_pid(env: RaceCarEnv, controller: CenterlinePIDController) -> Control:
    return controller.control(env.state)
