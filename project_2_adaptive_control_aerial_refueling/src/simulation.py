from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from system import AerialRefuelingPlant


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.01
    horizon: float = 45.0
    refueling_radius: float = 8.0
    initial_state: np.ndarray | None = None


def rk4_step(plant: AerialRefuelingPlant, time_value: float, state: np.ndarray, force: np.ndarray, dt: float) -> np.ndarray:
    k1 = plant.derivative(time_value, state, force)
    k2 = plant.derivative(time_value + 0.5 * dt, state + 0.5 * dt * k1, force)
    k3 = plant.derivative(time_value + 0.5 * dt, state + 0.5 * dt * k2, force)
    k4 = plant.derivative(time_value + dt, state + dt * k3, force)
    return state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def simulate(
    plant: AerialRefuelingPlant,
    controller,
    config: SimulationConfig,
) -> dict[str, np.ndarray]:
    initial_state = config.initial_state
    if initial_state is None:
        initial_state = np.array([95.0, -36.0, -1.0, 0.6], dtype=float)
    state = np.asarray(initial_state, dtype=float).copy()
    controller.reset()

    steps = int(config.horizon / config.dt)
    times = np.zeros(steps + 1)
    states = np.zeros((steps + 1, 4))
    forces = np.zeros((steps + 1, 2))
    masses = np.zeros(steps + 1)
    theta_true = np.zeros(steps + 1)
    theta_hat = np.zeros(steps + 1)
    mass_hat = np.zeros(steps + 1)
    filtered_errors = np.zeros((steps + 1, 2))
    position_norm = np.zeros(steps + 1)
    lyapunov = np.zeros(steps + 1)

    states[0] = state
    masses[0] = plant.mass(0.0)
    theta_true[0] = plant.theta(0.0)
    theta_hat[0] = getattr(controller, "theta_hat", np.nan)
    mass_hat[0] = getattr(controller, "mass_hat", np.nan)
    filtered_errors[0] = controller.filtered_error(state)
    position_norm[0] = np.linalg.norm(state[0:2])

    lyapunov[0] = compute_lyapunov(controller, filtered_errors[0], theta_true[0], theta_hat[0])

    for idx in range(steps):
        time_value = idx * config.dt
        output = controller.compute_force(state, config.dt)
        force = output.force
        state = rk4_step(plant, time_value, state, force, config.dt)

        next_idx = idx + 1
        next_time = next_idx * config.dt
        times[next_idx] = next_time
        states[next_idx] = state
        forces[next_idx] = force
        masses[next_idx] = plant.mass(next_time)
        theta_true[next_idx] = plant.theta(next_time)
        theta_hat[next_idx] = getattr(controller, "theta_hat", np.nan)
        mass_hat[next_idx] = getattr(controller, "mass_hat", np.nan)
        filtered_errors[next_idx] = controller.filtered_error(state)
        position_norm[next_idx] = np.linalg.norm(state[0:2])

        lyapunov[next_idx] = compute_lyapunov(controller, filtered_errors[next_idx], theta_true[next_idx], theta_hat[next_idx])

    return {
        "time": times,
        "state": states,
        "force": forces,
        "mass": masses,
        "theta_true": theta_true,
        "theta_hat": theta_hat,
        "mass_hat": mass_hat,
        "filtered_error": filtered_errors,
        "position_norm": position_norm,
        "lyapunov": lyapunov,
    }


def compute_lyapunov(controller, filtered_error: np.ndarray, theta_true: float, theta_hat: float) -> float:
    tracking_part = 0.5 * float(filtered_error @ filtered_error)
    gamma = getattr(controller, "gamma", None)
    if gamma is None or not np.isfinite(theta_hat):
        return tracking_part
    theta_error = theta_true - theta_hat
    return tracking_part + 0.5 * theta_error**2 / float(gamma)
