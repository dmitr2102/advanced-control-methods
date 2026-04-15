from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class AveragedLimitCycleController:
    """Zero-mean carrier controller designed from an averaged limit-cycle model."""

    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.18
    target_amplitude: float = 0.14
    target_frequency: float = 1.45
    carrier_frequency: float = 20.0
    energy_gain: float = 18.0
    smooth_gain: float = 40.0
    stiffness_floor: float = 0.35
    stiffness_ceiling: float = 4.5
    min_amplitude: float = 52.0
    max_amplitude: float = 112.0
    phase: float = 0.0
    last_time_value: float | None = None
    last_stiffness: float = 0.35
    last_amplitude: float = 52.0
    name: str = "averaged_limit_cycle"

    def reset(self) -> None:
        self.phase = 0.0
        self.last_time_value = None
        self.last_stiffness = self.target_frequency ** 2
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
        return 0.5 * error_phi_dot ** 2 + 0.5 * (self.target_frequency ** 2) * error_phi ** 2 + 0.5 * self.energy_gain * (energy_error ** 2)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        _, _, error_norm = self.tracking_error(time_value, state)
        energy = 0.5 * (phi_dot ** 2 + (self.target_frequency ** 2) * (phi ** 2))
        energy_error = energy - self.target_energy

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += self.carrier_frequency * dt

        # Averaged energy shaping:
        # E_dot = -d*phi_dot^2 - (k_eff - w_*^2) * phi * phi_dot.
        # Choosing k_eff = w_*^2 + k_E(E-E_*) sign(phi*phi_dot)
        # makes the extra term inject energy inside the orbit and dissipate it outside.
        signed_flow = math.tanh(self.smooth_gain * phi * phi_dot)
        target_stiffness = self.target_frequency ** 2 + self.energy_gain * energy_error * signed_flow
        target_stiffness = min(self.stiffness_ceiling, max(self.stiffness_floor, target_stiffness))

        amplitude_sq = 2.0 * (self.length ** 2) * (self.carrier_frequency ** 2)
        amplitude_sq *= (self.gravity / self.length + target_stiffness)
        amplitude = math.sqrt(max(0.0, amplitude_sq))
        amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))

        self.last_stiffness = target_stiffness
        self.last_amplitude = amplitude
        action = amplitude * math.cos(self.phase)

        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "carrier_frequency": self.carrier_frequency,
                "amplitude": amplitude,
                "target_stiffness": target_stiffness,
                "energy": energy,
                "energy_target": self.target_energy,
                "energy_error": energy_error,
                "tracking_error": error_norm,
                "lyapunov": self.lyapunov_value(time_value, state),
            },
        )
