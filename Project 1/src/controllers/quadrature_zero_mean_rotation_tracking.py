from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class QuadratureZeroMeanRotationTrackingController:
    """Zero-mean rotating-orbit controller using cosine/sine carrier channels."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_angular_speed: float = 1.6
    reference_phase: float = 0.0
    position_gain: float = 10.0
    rate_gain: float = 4.0
    regularization_eps: float = 0.08
    carrier_frequency: float = 24.0
    filter_tau: float = 0.12
    mixing_gain: float = 0.35
    channel_limit: float = 75.0
    phase: float = 0.0
    alpha_state: float = 0.0
    gamma_state: float = 0.0
    last_time_value: float | None = None
    name: str = "quadrature_zero_mean_rotation_tracking"

    def reset(self) -> None:
        self.phase = 0.0
        self.alpha_state = 0.0
        self.gamma_state = 0.0
        self.last_time_value = None

    def reference_state(self, time_value: float) -> tuple[float, float, float]:
        phi_ref = self.reference_phase + self.target_angular_speed * time_value
        phi_dot_ref = self.target_angular_speed
        phi_ddot_ref = 0.0
        return phi_ref, phi_dot_ref, phi_ddot_ref

    def tracking_error(self, time_value: float, state: tuple[float, float]) -> tuple[float, float, float]:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, _ = self.reference_state(time_value)
        error_phi = wrap_to_pi(phi - phi_ref)
        error_phi_dot = phi_dot - phi_dot_ref
        error_norm = math.sqrt(error_phi ** 2 + error_phi_dot ** 2)
        return error_phi, error_phi_dot, error_norm

    def lyapunov_value(self, time_value: float, state: tuple[float, float]) -> float:
        error_phi, error_phi_dot, _ = self.tracking_error(time_value, state)
        return 0.5 * error_phi_dot ** 2 + self.position_gain * (1.0 - math.cos(error_phi)) + 0.01 * (self.alpha_state ** 2 + self.gamma_state ** 2)

    def _regularized_sin(self, phi: float, phi_ref: float) -> float:
        sin_phi = math.sin(phi)
        if abs(sin_phi) >= self.regularization_eps:
            return sin_phi
        sign_source = phi if abs(phi) > 1e-9 else (phi_ref if abs(phi_ref) > 1e-9 else 1.0)
        return math.copysign(self.regularization_eps, sign_source)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, phi_ddot_ref = self.reference_state(time_value)
        error_phi, error_phi_dot, error_norm = self.tracking_error(time_value, state)

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += self.carrier_frequency * dt

        desired_phi_ddot = phi_ddot_ref - self.rate_gain * error_phi_dot - self.position_gain * math.sin(error_phi)
        sin_reg = self._regularized_sin(phi, phi_ref)
        u_des = self.length * (desired_phi_ddot + self.damping * phi_dot) / sin_reg - self.gravity

        # Project the desired slow action onto a zero-mean carrier with two channels.
        ref_phase = self.target_angular_speed * time_value
        alpha_target = self.mixing_gain * u_des * math.cos(ref_phase)
        gamma_target = self.mixing_gain * u_des * math.sin(ref_phase)
        if self.filter_tau > 1e-9 and dt > 0.0:
            self.alpha_state += dt * (-self.alpha_state + alpha_target) / self.filter_tau
            self.gamma_state += dt * (-self.gamma_state + gamma_target) / self.filter_tau
        else:
            self.alpha_state = alpha_target
            self.gamma_state = gamma_target

        self.alpha_state = max(-self.channel_limit, min(self.channel_limit, self.alpha_state))
        self.gamma_state = max(-self.channel_limit, min(self.channel_limit, self.gamma_state))
        action = self.alpha_state * math.cos(self.phase) + self.gamma_state * math.sin(self.phase)

        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "phi_ref": phi_ref,
                "phi_dot_ref": phi_dot_ref,
                "error_phi": error_phi,
                "error_phi_dot": error_phi_dot,
                "tracking_error": error_norm,
                "virtual_action": u_des,
                "alpha_channel": self.alpha_state,
                "gamma_channel": self.gamma_state,
                "carrier_frequency": self.carrier_frequency,
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
