from __future__ import annotations

from dataclasses import dataclass

from controllers.base import Controller
from system import KapitzaPendulumPlant


@dataclass
class SimulationState:
    time_value: float
    phi: float
    phi_dot: float
    action: float
    controller_name: str

    @property
    def as_tuple(self) -> tuple[float, float]:
        return self.phi, self.phi_dot


@dataclass
class SimulationConfig:
    dt: float = 1.0 / 240.0
    initial_phi: float = 0.22
    initial_phi_dot: float = 0.0
    max_abs_action: float = 140.0


class PendulumSimulator:
    """Time integration for the pendulum plant."""

    def __init__(self, plant: KapitzaPendulumPlant, config: SimulationConfig) -> None:
        self.plant = plant
        self.config = config
        self.state = SimulationState(
            time_value=0.0,
            phi=config.initial_phi,
            phi_dot=config.initial_phi_dot,
            action=0.0,
            controller_name="harmonic",
        )

    def reset(self) -> None:
        self.state = SimulationState(
            time_value=0.0,
            phi=self.config.initial_phi,
            phi_dot=self.config.initial_phi_dot,
            action=0.0,
            controller_name=self.state.controller_name,
        )

    def step(self, controller: Controller) -> SimulationState:
        output = controller.compute_action(self.state.time_value, self.state.as_tuple)
        action = max(-self.config.max_abs_action, min(self.config.max_abs_action, output.action))

        next_phi, next_phi_dot = self._rk4_step(self.state.as_tuple, action, self.config.dt)
        self.state = SimulationState(
            time_value=self.state.time_value + self.config.dt,
            phi=next_phi,
            phi_dot=next_phi_dot,
            action=action,
            controller_name=output.name,
        )
        return self.state

    def _rk4_step(self, state: tuple[float, float], action: float, dt: float) -> tuple[float, float]:
        def add_scaled(base: tuple[float, float], delta: tuple[float, float], scale: float) -> tuple[float, float]:
            return base[0] + scale * delta[0], base[1] + scale * delta[1]

        k1 = self.plant.p(state, action)
        k2 = self.plant.p(add_scaled(state, k1, dt / 2.0), action)
        k3 = self.plant.p(add_scaled(state, k2, dt / 2.0), action)
        k4 = self.plant.p(add_scaled(state, k3, dt), action)

        phi = state[0] + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        phi_dot = state[1] + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        return phi, phi_dot
