from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class HorizontalExcitationParameters:
    gravity: float = 9.81
    length: float = 1.0
    damping: float = 0.25


def horizontal_p(state: tuple[float, float], action: float, params: HorizontalExcitationParameters) -> tuple[float, float]:
    """Nonlinear pendulum model for horizontal suspension acceleration.

    State:
        s = [theta, theta_dot]^T
    Action:
        a = x_ddot
    Model:
        theta_ddot = -(g/l) sin(theta) - (a/l) cos(theta) - d theta_dot
    """

    theta, theta_dot = state
    theta_ddot = -(params.gravity / params.length) * math.sin(theta)
    theta_ddot -= (action / params.length) * math.cos(theta)
    theta_ddot -= params.damping * theta_dot
    return theta_dot, theta_ddot


def harmonic_horizontal_action(time_value: float, amplitude: float, frequency: float, phase: float = 0.0) -> float:
    return amplitude * math.cos(frequency * time_value + phase)


def averaged_effective_potential(theta: float, amplitude: float, frequency: float, params: HorizontalExcitationParameters) -> float:
    gravity_term = -(params.gravity / params.length) * math.cos(theta)
    shaping_term = -(amplitude ** 2) * (math.sin(theta) ** 2)
    shaping_term /= 4.0 * (params.length ** 2) * (frequency ** 2)
    return gravity_term + shaping_term


def averaged_side_equilibrium_cosine(amplitude: float, frequency: float, params: HorizontalExcitationParameters) -> float:
    return 2.0 * params.gravity * params.length * (frequency ** 2) / (amplitude ** 2)


def side_equilibria(amplitude: float, frequency: float, params: HorizontalExcitationParameters) -> tuple[float, float] | None:
    """Return the stable side equilibria of the averaged model, if they exist.

    The side equilibria satisfy:
        cos(theta*) = 2 g l omega^2 / alpha^2
    They exist only when the right-hand side is in (0, 1).
    """

    cosine_value = averaged_side_equilibrium_cosine(amplitude, frequency, params)
    if not (0.0 < cosine_value < 1.0):
        return None
    theta_star = math.acos(cosine_value)
    return theta_star, -theta_star


def horizontal_proximity_percent(amplitude: float, frequency: float, params: HorizontalExcitationParameters) -> float | None:
    """Return how close the side equilibrium is to a horizontal angle.

    100% means theta* = pi/2 exactly.
    0% means theta* = 0.
    """

    equilibria = side_equilibria(amplitude, frequency, params)
    if equilibria is None:
        return None
    theta_star = abs(equilibria[0])
    return 100.0 * theta_star / (0.5 * math.pi)

