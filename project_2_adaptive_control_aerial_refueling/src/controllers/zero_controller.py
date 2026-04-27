from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .base import ControllerOutput


@dataclass
class ZeroController:
    """Baseline controller with no corrective control action."""

    lambda_gains: np.ndarray
    name: str = "zero_controller"
    last_details: dict[str, float] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        self.lambda_matrix = np.diag(self.lambda_gains).astype(float)

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
        force = np.zeros(2, dtype=float)
        self.last_details = {
            "x": float(position[0]),
            "z": float(position[1]),
            "vx": float(velocity[0]),
            "vz": float(velocity[1]),
            "ex": float(error[0]),
            "ez": float(error[1]),
            "error_norm": float(np.linalg.norm(error)),
            "position_norm": float(np.linalg.norm(position)),
            "ux": 0.0,
            "uz": 0.0,
        }
        return ControllerOutput(name=self.name, force=force, details=self.last_details.copy())
