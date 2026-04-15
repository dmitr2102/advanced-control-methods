from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class QuadratureZeroMeanOrbitTrackingController:
    """Zero-mean orbital tracking using two quadrature carrier channels."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_amplitude: float = 0.14
    target_frequency: float = 1.45
    position_gain: float = 10.0
    rate_gain: float = 4.0
    orbit_gain: float = 8.0
    phase_gain: float = 1.0
    regularization_eps: float = 0.08
    carrier_frequency: float = 20.0
    filter_tau: float = 0.10
    mixing_gain: float = 0.85
    channel_limit: float = 90.0
    phase: float = 0.0
    alpha_state: float = 0.0
    gamma_state: float = 0.0
    last_time_value: float | None = None
    name: str = "quadrature_zero_mean_orbit_tracking"

    def reset(self) -> None:
        self.phase = 0.0
        self.alpha_state = 0.0
        self.gamma_state = 0.0
        self.last_time_value = None

    def reference_state(self, time_value: float) -> tuple[float, float, float]:
        phi_ref = self.target_amplitude * math.sin(self.target_frequency * time_value)
        phi_dot_ref = self.target_amplitude * self.target_frequency * math.cos(self.target_frequency * time_value)
        phi_ddot_ref = -(self.target_frequency ** 2) * phi_ref
        return phi_ref, phi_dot_ref, phi_ddot_ref

    def orbital_coordinates(self, state: tuple[float, float]) -> tuple[float, float]:
        phi, phi_dot = state
        scaled_rate = phi_dot / max(self.target_frequency, 1e-6)
        radius = math.sqrt(phi * phi + scaled_rate * scaled_rate)
        phase = math.atan2(phi, scaled_rate)
        return radius, phase

    def tracking_error(self, time_value: float, state: tuple[float, float]) -> tuple[float, float, float, float, float]:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, _ = self.reference_state(time_value)
        e1 = phi - phi_ref
        e2 = phi_dot - phi_dot_ref
        radius, phase = self.orbital_coordinates(state)
        radial_error = radius - self.target_amplitude
        phase_error = wrap_to_pi(phase - self.target_frequency * time_value)
        error_norm = math.sqrt(e1 ** 2 + (e2 / max(self.target_frequency, 1e-6)) ** 2)
        return e1, e2, error_norm, radial_error, phase_error

    def lyapunov_value(self, time_value: float, state: tuple[float, float]) -> float:
        e1, e2, _, radial_error, phase_error = self.tracking_error(time_value, state)
        return (
            0.5 * e2 ** 2
            + 0.5 * (self.target_frequency ** 2) * e1 ** 2
            + 0.5 * self.orbit_gain * radial_error ** 2
            + 0.5 * self.phase_gain * phase_error ** 2
            + 0.01 * (self.alpha_state ** 2 + self.gamma_state ** 2)
        )

    def _regularized_sin(self, phi: float, phi_ref: float) -> float:
        sin_phi = math.sin(phi)
        if abs(sin_phi) >= self.regularization_eps:
            return sin_phi
        if abs(phi_ref) > 1e-6:
            return math.copysign(self.regularization_eps, phi_ref)
        return self.regularization_eps

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, phi_ddot_ref = self.reference_state(time_value)
        e1, e2, error_norm, radial_error, phase_error = self.tracking_error(time_value, state)

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += self.carrier_frequency * dt

        desired_phi_ddot = (
            phi_ddot_ref
            - self.rate_gain * e2
            - self.position_gain * e1
            - self.orbit_gain * radial_error * phi_dot
            - self.phase_gain * phase_error
        )

        sin_reg = self._regularized_sin(phi, phi_ref)
        u_des = self.length * (desired_phi_ddot + self.damping * phi_dot) / sin_reg - self.gravity

        # Project the virtual control into two zero-mean quadrature channels.
        ref_phase = self.target_frequency * time_value
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
                "error_phi": e1,
                "error_phi_dot": e2,
                "tracking_error": error_norm,
                "radial_error": radial_error,
                "phase_error": phase_error,
                "virtual_action": u_des,
                "alpha_channel": self.alpha_state,
                "gamma_channel": self.gamma_state,
                "carrier_frequency": self.carrier_frequency,
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
