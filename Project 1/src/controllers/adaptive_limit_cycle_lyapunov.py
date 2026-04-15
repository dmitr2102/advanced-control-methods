from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class AdaptiveLimitCycleLyapunovController:
    """Zero-mean carrier controller with state-dependent amplitude and frequency."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_amplitude: float = 0.14
    target_frequency: float = 1.45
    position_gain: float = 12.0
    rate_gain: float = 5.0
    energy_gain: float = 10.0
    energy_weight: float = 3.0
    base_carrier_frequency: float = 20.0
    angle_frequency_gain: float = 8.0
    rate_frequency_gain: float = 0.5
    energy_frequency_gain: float = 10.0
    min_carrier_frequency: float = 16.0
    max_carrier_frequency: float = 28.0
    stiffness_floor: float = 0.5
    stiffness_ceiling: float = 12.0
    regularization_eps: float = 0.05
    min_amplitude: float = 68.0
    max_amplitude: float = 128.0
    phase: float = 0.0
    last_time_value: float | None = None
    last_carrier_frequency: float = 18.0
    last_amplitude: float = 68.0
    name: str = "adaptive_limit_cycle_lyapunov"

    def reset(self) -> None:
        self.phase = 0.0
        self.last_time_value = None
        self.last_carrier_frequency = self.base_carrier_frequency
        self.last_amplitude = self.min_amplitude

    @property
    def target_energy(self) -> float:
        return 0.5 * (self.target_frequency ** 2) * (self.target_amplitude ** 2)

    def reference_state(self, time_value: float) -> tuple[float, float, float]:
        phi_ref = self.target_amplitude * math.sin(self.target_frequency * time_value)
        phi_dot_ref = self.target_amplitude * self.target_frequency * math.cos(self.target_frequency * time_value)
        phi_ddot_ref = -(self.target_frequency ** 2) * phi_ref
        return phi_ref, phi_dot_ref, phi_ddot_ref

    def tracking_error(self, time_value: float, state: tuple[float, float]) -> tuple[float, float, float]:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, _ = self.reference_state(time_value)
        error_phi = phi - phi_ref
        error_phi_dot = phi_dot - phi_dot_ref
        error_norm = math.sqrt(error_phi ** 2 + (error_phi_dot / max(self.target_frequency, 1e-6)) ** 2)
        return error_phi, error_phi_dot, error_norm

    def lyapunov_value(self, time_value: float, state: tuple[float, float]) -> float:
        error_phi, error_phi_dot, _ = self.tracking_error(time_value, state)
        phi, phi_dot = state
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy
        return (
            0.5 * (error_phi_dot ** 2)
            + 0.5 * (self.target_frequency ** 2) * (error_phi ** 2)
            + 0.5 * self.energy_weight * (energy_error ** 2)
        )

    def _regularized_phi(self, phi: float, phi_ref: float) -> float:
        if abs(phi) >= self.regularization_eps:
            return phi
        if abs(phi_ref) >= 1e-6:
            return math.copysign(self.regularization_eps, phi_ref)
        return self.regularization_eps

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        phi_ref, phi_dot_ref, phi_ddot_ref = self.reference_state(time_value)
        error_phi, error_phi_dot, error_norm = self.tracking_error(time_value, state)
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy

        carrier_frequency = (
            self.base_carrier_frequency
            + self.angle_frequency_gain * abs(error_phi)
            + self.rate_frequency_gain * abs(error_phi_dot)
            + self.energy_frequency_gain * abs(energy_error)
        )
        carrier_frequency = min(self.max_carrier_frequency, max(self.min_carrier_frequency, carrier_frequency))

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += carrier_frequency * dt

        desired_phi_ddot = (
            phi_ddot_ref
            - self.rate_gain * error_phi_dot
            - self.position_gain * error_phi
            - self.energy_gain * energy_error * phi_dot
        )

        phi_reg = self._regularized_phi(phi, phi_ref)
        target_stiffness = -(desired_phi_ddot + self.damping * phi_dot) / phi_reg
        target_stiffness = min(self.stiffness_ceiling, max(self.stiffness_floor, target_stiffness))

        amplitude_sq = 2.0 * (self.length ** 2) * (carrier_frequency ** 2)
        amplitude_sq *= (self.gravity / self.length + target_stiffness)
        amplitude = math.sqrt(max(0.0, amplitude_sq))
        amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))
        self.last_carrier_frequency = carrier_frequency
        self.last_amplitude = amplitude

        action = amplitude * math.cos(self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "carrier_frequency": carrier_frequency,
                "amplitude": amplitude,
                "phi_ref": phi_ref,
                "phi_dot_ref": phi_dot_ref,
                "error_phi": error_phi,
                "error_phi_dot": error_phi_dot,
                "tracking_error": error_norm,
                "energy": energy,
                "energy_target": self.target_energy,
                "energy_error": energy_error,
                "stiffness": target_stiffness,
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
