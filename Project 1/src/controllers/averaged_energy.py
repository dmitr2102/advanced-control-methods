from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class AveragedEnergyController:
    """State-dependent amplitude shaping based on the averaged Kapitza energy."""

    gravity: float = 9.81
    length: float = 1.0
    carrier_frequency: float = 24.0
    target_stiffness: float = 10.4
    angle_gain: float = 6.0
    rate_gain: float = 0.8
    max_amplitude: float = 111.0
    min_amplitude: float = 63.0
    phase: float = 0.0
    last_time_value: float | None = None
    name: str = "averaged_energy"

    def reset(self) -> None:
        self.phase = 0.0
        self.last_time_value = None

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state

        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += self.carrier_frequency * dt

        target_stiffness = self.target_stiffness
        target_stiffness += self.angle_gain * (1.0 - math.cos(phi))
        target_stiffness += self.rate_gain * (phi_dot ** 2)

        required_amplitude_sq = 2.0 * (self.length ** 2) * (self.carrier_frequency ** 2)
        required_amplitude_sq *= (self.gravity / self.length + target_stiffness)
        amplitude = math.sqrt(max(0.0, required_amplitude_sq))
        amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))

        action = amplitude * math.cos(self.phase)

        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "carrier_frequency": self.carrier_frequency,
                "amplitude": amplitude,
                "target_stiffness": self.target_stiffness,
                "phi": phi,
                "phi_dot": phi_dot,
            },
        )
