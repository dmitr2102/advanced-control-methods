from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class HarmonicController:
    """Open-loop harmonic excitation of the suspension point."""

    amplitude: float = 116.5
    frequency: float = 25.3
    phase: float = 0.0
    enabled: bool = True
    name: str = "harmonic"

    def reset(self) -> None:
        self.phase = 0.0

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        del state
        if not self.enabled:
            action = 0.0
        else:
            action = self.amplitude * math.cos(self.frequency * time_value + self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "amplitude": self.amplitude,
                "frequency": self.frequency,
                "phase": self.phase,
            },
        )
