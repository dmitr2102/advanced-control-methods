from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class DirectLyapunovController:
    """Direct Lyapunov controller on the exact vertical pendulum dynamics.

    Candidate Lyapunov function:
        V(phi, phi_dot) = 0.5 * phi_dot^2 + k_p * (1 - cos(phi))

    Control law:
        a = -g - l k_p - l k_c phi_dot sin(phi)

    Then the exact closed-loop dynamics satisfies
        V_dot = -(d + k_c sin^2(phi)) * phi_dot^2 <= 0.
    """

    gravity: float = 9.81
    length: float = 1.0
    stiffness_gain: float = 2.0
    damping_shaping_gain: float = 80.0
    name: str = "direct_lyapunov"

    def reset(self) -> None:
        return None

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        del time_value
        phi, phi_dot = state
        action = -self.gravity - self.length * self.stiffness_gain
        action -= self.length * self.damping_shaping_gain * phi_dot * math.sin(phi)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "stiffness_gain": self.stiffness_gain,
                "damping_shaping_gain": self.damping_shaping_gain,
            },
        )
