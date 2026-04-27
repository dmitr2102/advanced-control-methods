from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RefuelingPlantParameters:
    initial_mass: float = 12000.0
    fuel_flow_nominal: float = 42.0
    fuel_flow_variation: float = 10.0
    gravity: float = 9.81
    damping_x: float = 2300.0
    damping_z: float = 2600.0


class AerialRefuelingPlant:
    """2D trimmed-flight perturbation model for receiver station keeping."""

    def __init__(self, params: RefuelingPlantParameters) -> None:
        self.params = params
        self.vertical_unit = np.array([0.0, 1.0], dtype=float)
        self.damping = np.diag([params.damping_x, params.damping_z]).astype(float)

    def fuel_flow(self, time_value: float) -> float:
        """True fuel mass flow q(t), kg/s."""
        return self.params.fuel_flow_nominal + self.params.fuel_flow_variation * np.sin(0.22 * time_value)

    def mass(self, time_value: float) -> float:
        """Closed-form integral of the fuel-flow profile."""
        q0 = self.params.fuel_flow_nominal
        q1 = self.params.fuel_flow_variation
        omega = 0.22
        return self.params.initial_mass + q0 * time_value + q1 * (1.0 - np.cos(omega * time_value)) / omega

    def theta(self, time_value: float) -> float:
        return 1.0 / self.mass(time_value)

    def phi(self, state: np.ndarray, force: np.ndarray) -> np.ndarray:
        velocity = state[2:4]
        return force - self.damping @ velocity + self.params.initial_mass * self.params.gravity * self.vertical_unit

    def derivative(self, time_value: float, state: np.ndarray, force: np.ndarray) -> np.ndarray:
        velocity = state[2:4]
        theta = self.theta(time_value)
        acceleration = theta * self.phi(state, force) - self.params.gravity * self.vertical_unit
        return np.array([velocity[0], velocity[1], acceleration[0], acceleration[1]], dtype=float)
