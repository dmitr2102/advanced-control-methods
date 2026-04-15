from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class DirectEllipticRotationTrackingController:
    """Direct controller for a full-rotation regime with an elliptic wrapped phase portrait."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    base_rotation_speed: float = 1.6
    phase_amplitude: float = 0.35
    modulation_frequency: float = 3.0
    reference_phase: float = 0.0
    position_gain: float = 12.0
    rate_gain: float = 5.0
    max_abs_action: float = 140.0
    regularization_eps: float = 0.08
    name: str = "direct_elliptic_rotation_tracking"

    @property
    def vertical_semi_axis(self) -> float:
        return self.phase_amplitude * self.modulation_frequency

    def reference_state(self, time_value: float) -> tuple[float, float, float]:
        phi_ref = (
            self.reference_phase
            + self.base_rotation_speed * time_value
            + self.phase_amplitude * math.sin(self.modulation_frequency * time_value)
        )
        phi_dot_ref = self.base_rotation_speed + self.vertical_semi_axis * math.cos(self.modulation_frequency * time_value)
        phi_ddot_ref = -self.phase_amplitude * (self.modulation_frequency ** 2) * math.sin(self.modulation_frequency * time_value)
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
        return 0.5 * error_phi_dot ** 2 + self.position_gain * (1.0 - math.cos(error_phi))

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, phi_ddot_ref = self.reference_state(time_value)
        error_phi, error_phi_dot, error_norm = self.tracking_error(time_value, state)

        desired_phi_ddot = phi_ddot_ref - self.rate_gain * error_phi_dot - self.position_gain * math.sin(error_phi)

        sin_phi = math.sin(phi)
        if abs(sin_phi) < self.regularization_eps:
            sign_source = phi if abs(phi) > 1e-9 else (phi_ref if abs(phi_ref) > 1e-9 else 1.0)
            sin_phi = math.copysign(self.regularization_eps, sign_source)

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
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
