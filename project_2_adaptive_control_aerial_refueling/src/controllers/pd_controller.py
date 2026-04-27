from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .base import ControllerOutput


@dataclass
class NominalMassPDController:
    """PD baseline that uses the nominal mass but has no adaptation."""

    damping: np.ndarray
    nominal_mass: float
    lambda_gains: np.ndarray
    kp_gains: np.ndarray
    kd_gains: np.ndarray
    max_force: float
    name: str = "nominal_mass_pd"
    last_details: dict[str, float] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        self.lambda_matrix = np.diag(self.lambda_gains).astype(float)
        self.kp_matrix = np.diag(self.kp_gains).astype(float)
        self.kd_matrix = np.diag(self.kd_gains).astype(float)

    def reset(self) -> None:
        self.last_details = {}

    def filtered_error(self, state: np.ndarray) -> np.ndarray:
        position = state[0:2]
        velocity = state[2:4]
        return velocity + self.lambda_matrix @ position

    def compute_force(self, state: np.ndarray, dt: float) -> ControllerOutput:
        position = state[0:2]
        velocity = state[2:4]
        error = self.filtered_error(state)

        desired_acceleration = -self.kp_matrix @ position - self.kd_matrix @ velocity
        force = self.damping @ velocity + self.nominal_mass * desired_acceleration
        force = np.clip(force, -self.max_force, self.max_force)

        self.last_details = {
            "x": float(position[0]),
            "z": float(position[1]),
            "vx": float(velocity[0]),
            "vz": float(velocity[1]),
            "ex": float(error[0]),
            "ez": float(error[1]),
            "error_norm": float(np.linalg.norm(error)),
            "position_norm": float(np.linalg.norm(position)),
            "ax_des": float(desired_acceleration[0]),
            "az_des": float(desired_acceleration[1]),
            "ux": float(force[0]),
            "uz": float(force[1]),
        }
        return ControllerOutput(name=self.name, force=force, details=self.last_details.copy())
