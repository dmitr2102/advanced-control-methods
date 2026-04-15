from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class ResonantZeroMeanOrbitTrackingController:
    """Zero-mean resonant controller for sustaining an oscillatory orbit."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_amplitude: float = 0.14
    target_frequency: float = 1.45
    base_carrier_frequency: float = 1.45
    carrier_frequency_gain: float = 0.35
    energy_gain: float = 220.0
    phase_lead: float = 0.0
    min_amplitude: float = 0.0
    max_amplitude: float = 48.0
    phase: float = 0.0
    last_time_value: float | None = None
    last_carrier_frequency: float = 1.45
    last_amplitude: float = 0.0
    name: str = "resonant_zero_mean_orbit_tracking"

    @property
    def target_energy(self) -> float:
        return 0.5 * (self.target_frequency ** 2) * (self.target_amplitude ** 2)

    def reset(self) -> None:
        self.phase = 0.0
        self.last_time_value = None
        self.last_carrier_frequency = self.base_carrier_frequency
        self.last_amplitude = self.min_amplitude

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
        norm = math.sqrt(e1 * e1 + (e2 / max(self.target_frequency, 1e-6)) ** 2)
        return e1, e2, norm

    def lyapunov_value(self, time_value: float, state: tuple[float, float]) -> float:
        e1, e2, _ = self.tracking_error(time_value, state)
        phi, phi_dot = state
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy
        return 0.5 * e2 ** 2 + 0.5 * (self.target_frequency ** 2) * e1 ** 2 + 0.5 * self.energy_gain * (energy_error ** 2)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy
        e1, e2, error_norm = self.tracking_error(time_value, state)

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value

        carrier_frequency = self.base_carrier_frequency + self.carrier_frequency_gain * math.tanh(12.0 * energy_error)
        carrier_frequency = max(0.2, carrier_frequency)
        self.phase += carrier_frequency * dt

        # Resonant energy pumping: inject energy with the sign of phi*phi_dot.
        # Inside the target orbit, the amplitude grows; outside it decays.
        signed_pumping = math.tanh(40.0 * phi * phi_dot)
        amplitude = self.energy_gain * abs(energy_error)
        amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))

        # Carrier phase is synchronized with the orbital energy flow.
        phase_shift = self.phase_lead if signed_pumping >= 0.0 else self.phase_lead + math.pi
        action = amplitude * math.cos(self.phase + phase_shift)

        self.last_carrier_frequency = carrier_frequency
        self.last_amplitude = amplitude
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "carrier_frequency": carrier_frequency,
                "amplitude": amplitude,
                "energy": energy,
                "energy_target": self.target_energy,
                "energy_error": energy_error,
                "signed_pumping": signed_pumping,
                "tracking_error": error_norm,
                "lyapunov": self.lyapunov_value(time_value, state),
                "error_phi": e1,
                "error_phi_dot": e2,
            },
        )
