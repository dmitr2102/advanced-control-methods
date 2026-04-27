from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .base import ControllerOutput


@dataclass
class AdaptiveInverseMassController:
    """Certainty-equivalence adaptive controller with inverse-mass estimation."""

    damping: np.ndarray
    initial_mass: float
    gravity: float
    lambda_gains: np.ndarray
    k_gains: np.ndarray
    gamma: float
    mass_hat_initial: float
    mass_min: float
    mass_max: float
    max_force: float
    name: str = "adaptive_inverse_mass"
    theta_hat: float = field(init=False)
    last_details: dict[str, float] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        self.vertical_unit = np.array([0.0, 1.0], dtype=float)
        self.lambda_matrix = np.diag(self.lambda_gains).astype(float)
        self.k_matrix = np.diag(self.k_gains).astype(float)
        self.theta_min = 1.0 / self.mass_max
        self.theta_max = 1.0 / self.mass_min
        self.reset()

    def reset(self) -> None:
        self.theta_hat = float(np.clip(1.0 / self.mass_hat_initial, self.theta_min, self.theta_max))
        self.last_details = {}

    @property
    def mass_hat(self) -> float:
        return 1.0 / self.theta_hat

    def filtered_error(self, state: np.ndarray) -> np.ndarray:
        position = state[0:2]
        velocity = state[2:4]
        return velocity + self.lambda_matrix @ position

    def compute_force(self, state: np.ndarray, dt: float) -> ControllerOutput:
        position = state[0:2]
        velocity = state[2:4]
        error = self.filtered_error(state)

        desired_term = -self.k_matrix @ error - self.lambda_matrix @ velocity + self.gravity * self.vertical_unit
        force = self.damping @ velocity - self.initial_mass * self.gravity * self.vertical_unit
        force = force + desired_term / self.theta_hat
        force = np.clip(force, -self.max_force, self.max_force)

        phi = force - self.damping @ velocity + self.initial_mass * self.gravity * self.vertical_unit
        theta_hat_dot = self.gamma * float(error @ phi)
        self.theta_hat = float(np.clip(self.theta_hat + dt * theta_hat_dot, self.theta_min, self.theta_max))

        lyapunov = 0.5 * float(error @ error)
        self.last_details = {
            "x": float(position[0]),
            "z": float(position[1]),
            "vx": float(velocity[0]),
            "vz": float(velocity[1]),
            "ex": float(error[0]),
            "ez": float(error[1]),
            "error_norm": float(np.linalg.norm(error)),
            "position_norm": float(np.linalg.norm(position)),
            "theta_hat": float(self.theta_hat),
            "mass_hat": float(self.mass_hat),
            "theta_hat_dot": float(theta_hat_dot),
            "lyapunov_tracking": lyapunov,
            "ux": float(force[0]),
            "uz": float(force[1]),
        }
        return ControllerOutput(name=self.name, force=force, details=self.last_details.copy())
