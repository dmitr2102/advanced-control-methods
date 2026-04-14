from __future__ import annotations

from dataclasses import dataclass

from controllers.base import Controller
from horizontal_excitation import HorizontalExcitationParameters, horizontal_p


@dataclass
class HorizontalSimulationState:
    time_value: float
    theta: float
    theta_dot: float
    action: float
    controller_name: str

    @property
    def as_tuple(self) -> tuple[float, float]:
        return self.theta, self.theta_dot


@dataclass
class HorizontalSimulationConfig:
    dt: float = 1.0 / 240.0
    initial_theta: float = 1.0
    initial_theta_dot: float = 0.0
    max_abs_action: float = 1200.0


class HorizontalPendulumSimulator:
    """Time integration for the pendulum with horizontal suspension excitation."""

    def __init__(self, params: HorizontalExcitationParameters, config: HorizontalSimulationConfig) -> None:
        self.params = params
        self.config = config
        self.state = HorizontalSimulationState(
            time_value=0.0,
            theta=config.initial_theta,
            theta_dot=config.initial_theta_dot,
            action=0.0,
            controller_name="horizontal_harmonic",
        )

    def reset(self) -> None:
        self.state = HorizontalSimulationState(
            time_value=0.0,
            theta=self.config.initial_theta,
            theta_dot=self.config.initial_theta_dot,
            action=0.0,
            controller_name=self.state.controller_name,
        )

    def step(self, controller: Controller) -> HorizontalSimulationState:
        output = controller.compute_action(self.state.time_value, self.state.as_tuple)
        action = max(-self.config.max_abs_action, min(self.config.max_abs_action, output.action))
        next_theta, next_theta_dot = self._rk4_step(self.state.as_tuple, action, self.config.dt)
        self.state = HorizontalSimulationState(
            time_value=self.state.time_value + self.config.dt,
            theta=next_theta,
            theta_dot=next_theta_dot,
            action=action,
            controller_name=output.name,
        )
        return self.state

    def _rk4_step(self, state: tuple[float, float], action: float, dt: float) -> tuple[float, float]:
        def add_scaled(base: tuple[float, float], delta: tuple[float, float], scale: float) -> tuple[float, float]:
            return base[0] + scale * delta[0], base[1] + scale * delta[1]

        k1 = horizontal_p(state, action, self.params)
        k2 = horizontal_p(add_scaled(state, k1, dt / 2.0), action, self.params)
        k3 = horizontal_p(add_scaled(state, k2, dt / 2.0), action, self.params)
        k4 = horizontal_p(add_scaled(state, k3, dt), action, self.params)
        theta = state[0] + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        theta_dot = state[1] + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        return theta, theta_dot
