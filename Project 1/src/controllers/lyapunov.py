from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class LyapunovController:
    """Lyapunov controller based on a target closed-loop averaged dynamics."""

    gravity: float = 9.81
    length: float = 1.0
    carrier_frequency: float = 18.0
    stiffness_linear: float = 0.35
    stiffness_cubic: float = 2.35
    max_amplitude: float = 104.0
    min_amplitude: float = 62.5
    phase: float = 0.0
    last_time_value: float | None = None
    name: str = "lyapunov"

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

        # Target local averaged closed-loop dynamics:
        # phi_ddot + d phi_dot + (k0 + k1 phi^2) phi = 0.
        target_stiffness = self.stiffness_linear + self.stiffness_cubic * (phi ** 2)

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
                "stiffness_linear": self.stiffness_linear,
                "stiffness_cubic": self.stiffness_cubic,
                "phi": phi,
                "phi_dot": phi_dot,
            },
        )


LyapunovPlaceholderController = LyapunovController
