from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class DirectOrbitTrackingController:
    """Direct nonlinear orbital-tracking controller for a prescribed pendulum oscillation."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_amplitude: float = 0.14
    target_frequency: float = 1.45
    position_gain: float = 12.0
    rate_gain: float = 4.0
    orbit_gain: float = 10.0
    phase_gain: float = 0.0
    regularization_eps: float = 0.08
    max_abs_action: float = 140.0
    name: str = "direct_orbit_tracking"

    @property
    def target_energy(self) -> float:
        return 0.5 * (self.target_frequency ** 2) * (self.target_amplitude ** 2)

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

    def tracking_error(self, time_value: float, state: tuple[float, float]) -> tuple[float, float, float, float]:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, _ = self.reference_state(time_value)
        error_phi = phi - phi_ref
        error_phi_dot = phi_dot - phi_dot_ref

        radius, phase = self.orbital_coordinates(state)
        phase_ref = self.target_frequency * time_value
        phase_error = wrap_to_pi(phase - phase_ref)
        error_norm = math.sqrt(error_phi ** 2 + (error_phi_dot / max(self.target_frequency, 1e-6)) ** 2)
        return error_phi, error_phi_dot, error_norm, phase_error

    def lyapunov_value(self, time_value: float, state: tuple[float, float]) -> float:
        error_phi, error_phi_dot, _, phase_error = self.tracking_error(time_value, state)
        radius, _ = self.orbital_coordinates(state)
        radial_error = radius - self.target_amplitude
        return (
            0.5 * error_phi_dot ** 2
            + 0.5 * (self.target_frequency ** 2) * error_phi ** 2
            + 0.5 * self.orbit_gain * radial_error ** 2
            + 0.5 * self.phase_gain * phase_error ** 2
        )

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, phi_ddot_ref = self.reference_state(time_value)
        error_phi, error_phi_dot, error_norm, phase_error = self.tracking_error(time_value, state)
        radius, _ = self.orbital_coordinates(state)
        radial_error = radius - self.target_amplitude

        # Desired acceleration combines trajectory tracking with orbital and phase attraction.
        desired_phi_ddot = (
            phi_ddot_ref
            - self.rate_gain * error_phi_dot
            - self.position_gain * error_phi
            - self.orbit_gain * radial_error * phi_dot
            - self.phase_gain * phase_error
        )

        sin_phi = math.sin(phi)
        if abs(sin_phi) < self.regularization_eps:
            sin_phi = math.copysign(self.regularization_eps, sin_phi if abs(sin_phi) > 1e-9 else (phi_ref if abs(phi_ref) > 1e-9 else 1.0))

        action = self.length * (desired_phi_ddot + self.damping * phi_dot) / sin_phi - self.gravity
        action = max(-self.max_abs_action, min(self.max_abs_action, action))

        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "phi_ref": phi_ref,
                "phi_dot_ref": phi_dot_ref,
                "error_phi": error_phi,
                "error_phi_dot": error_phi_dot,
                "tracking_error": error_norm,
                "radial_error": radial_error,
                "phase_error": phase_error,
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
