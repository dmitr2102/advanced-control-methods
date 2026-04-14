from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PendulumParameters:
    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.25


class KapitzaPendulumPlant:
    """Nonlinear pendulum plant with vertical suspension acceleration input."""

    def __init__(self, params: PendulumParameters) -> None:
        self.params = params

    def p(self, state: tuple[float, float], action: float) -> tuple[float, float]:
        phi, phi_dot = state
        gravity_term = (self.params.gravity + action) / self.params.length
        phi_ddot = gravity_term * math.sin(phi) - self.params.damping * phi_dot
        return phi_dot, phi_ddot
