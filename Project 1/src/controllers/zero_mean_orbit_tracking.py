from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class ZeroMeanOrbitTrackingController:
    """Orbital tracking with a zero-mean carrier input."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_amplitude: float = 0.14
    target_frequency: float = 1.45
    position_gain: float = 10.0
    rate_gain: float = 4.5
    energy_gain: float = 8.0
    regularization_eps: float = 0.08
    filter_tau: float = 0.18
    filter_state: float = 0.0
    carrier_base_frequency: float = 20.0
    carrier_angle_gain: float = 9.0
    carrier_rate_gain: float = 1.2
    carrier_energy_gain: float = 16.0
    min_carrier_frequency: float = 16.0
    max_carrier_frequency: float = 30.0
    min_amplitude: float = 30.0
    max_amplitude: float = 140.0
    amplitude_gain: float = 1.0
    beta_softening: float = 12.0
    phase: float = 0.0
    last_time_value: float | None = None
    last_carrier_frequency: float = 20.0
    last_amplitude: float = 30.0
    last_beta: float = 0.0
    name: str = "zero_mean_orbit_tracking"

    def reset(self) -> None:
        self.filter_state = 0.0
        self.phase = 0.0
        self.last_time_value = None
        self.last_carrier_frequency = self.carrier_base_frequency
        self.last_amplitude = self.min_amplitude
        self.last_beta = 0.0

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
        e1 = phi - phi_ref
        e2 = phi_dot - phi_dot_ref
        norm = math.sqrt(e1 ** 2 + (e2 / max(self.target_frequency, 1e-6)) ** 2)
        return e1, e2, norm

    def lyapunov_value(self, time_value: float, state: tuple[float, float]) -> float:
        e1, e2, _ = self.tracking_error(time_value, state)
        phi, phi_dot = state
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy
        filter_error = self.filter_state
        return 0.5 * e2 ** 2 + 0.5 * (self.target_frequency ** 2) * e1 ** 2 + 0.5 * self.energy_gain * (energy_error ** 2) + 0.03 * filter_error ** 2

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
        e1, e2, error_norm = self.tracking_error(time_value, state)
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value

        sin_reg = self._regularized_sin(phi, phi_ref)
        u_des = self.length * (
            phi_ddot_ref - self.rate_gain * e2 - self.position_gain * e1 - self.energy_gain * energy_error * phi_dot + self.damping * phi_dot
        ) / sin_reg - self.gravity

        # Slow internal model of the virtual desired acceleration.
        if self.filter_tau > 1e-9 and dt > 0.0:
            self.filter_state += dt * (-self.filter_state + u_des) / self.filter_tau
        else:
            self.filter_state = u_des

        carrier_frequency = (
            self.carrier_base_frequency
            + self.carrier_angle_gain * abs(e1)
            + self.carrier_rate_gain * abs(e2)
            + self.carrier_energy_gain * abs(energy_error)
        )
        carrier_frequency = min(self.max_carrier_frequency, max(self.min_carrier_frequency, carrier_frequency))
        self.phase += carrier_frequency * dt

        amplitude = self.min_amplitude + self.amplitude_gain * abs(self.filter_state)
        amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))

        beta = math.atan2(self.beta_softening, self.filter_state)
        action = amplitude * math.cos(self.phase + beta)

        self.last_carrier_frequency = carrier_frequency
        self.last_amplitude = amplitude
        self.last_beta = beta

        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "phi_ref": phi_ref,
                "phi_dot_ref": phi_dot_ref,
                "error_phi": e1,
                "error_phi_dot": e2,
                "tracking_error": error_norm,
                "energy": energy,
                "energy_target": self.target_energy,
                "energy_error": energy_error,
                "virtual_action": u_des,
                "filter_state": self.filter_state,
                "carrier_frequency": carrier_frequency,
                "amplitude": amplitude,
                "beta": beta,
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
