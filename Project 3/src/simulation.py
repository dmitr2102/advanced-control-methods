from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from system import CablePulleyPlant


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.002
    horizon: float = 8.0
    initial_state: np.ndarray | None = None


def rk4_step(
    plant: CablePulleyPlant,
    time_value: float,
    state: np.ndarray,
    voltage: float,
    dt: float,
) -> np.ndarray:
    k1 = plant.derivative(time_value, state, voltage)
    k2 = plant.derivative(time_value + 0.5 * dt, state + 0.5 * dt * k1, voltage)
    k3 = plant.derivative(time_value + 0.5 * dt, state + 0.5 * dt * k2, voltage)
    k4 = plant.derivative(time_value + dt, state + dt * k3, voltage)
    return state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def simulate(
    plant: CablePulleyPlant,
    controller,
    config: SimulationConfig,
) -> dict[str, np.ndarray]:
    if config.initial_state is None:
        state = np.zeros(5, dtype=float)
    else:
        state = np.asarray(config.initial_state, dtype=float).copy()

    controller.reset()
    steps = int(config.horizon / config.dt)
    times = np.linspace(0.0, steps * config.dt, steps + 1)
    states = np.zeros((steps + 1, 5), dtype=float)
    voltages = np.zeros(steps + 1, dtype=float)
    references = np.zeros(steps + 1, dtype=float)
    deformations = np.zeros(steps + 1, dtype=float)
    deformation_rates = np.zeros(steps + 1, dtype=float)
    delta_tensions = np.zeros(steps + 1, dtype=float)
    tension_1 = np.zeros(steps + 1, dtype=float)
    tension_2 = np.zeros(steps + 1, dtype=float)
    target_torques = np.zeros(steps + 1, dtype=float)
    motor_cable_torques = np.zeros(steps + 1, dtype=float)
    rod_gravity_torques = np.zeros(steps + 1, dtype=float)
    detail_records: list[dict[str, float]] = [{} for _ in range(steps + 1)]

    states[0] = state
    references[0] = controller.reference_angle(0.0)
    deformations[0], deformation_rates[0] = plant.cable_kinematics(state)
    moments = plant.moments(state)
    delta_tensions[0] = moments["delta_tension"]
    tension_1[0] = moments["tension_1"]
    tension_2[0] = moments["tension_2"]
    target_torques[0] = moments["target_torque"]
    motor_cable_torques[0] = moments["motor_cable_torque"]
    rod_gravity_torques[0] = moments["rod_gravity_torque"]

    for idx in range(steps):
        time_value = times[idx]
        output = controller.compute_action(time_value, state, config.dt)
        voltage = output.action
        detail_records[idx] = output.details.copy()
        state = rk4_step(plant, time_value, state, voltage, config.dt)

        next_idx = idx + 1
        states[next_idx] = state
        voltages[idx] = voltage
        voltages[next_idx] = voltage
        references[next_idx] = controller.reference_angle(times[next_idx])
        deformations[next_idx], deformation_rates[next_idx] = plant.cable_kinematics(state)
        moments = plant.moments(state)
        delta_tensions[next_idx] = moments["delta_tension"]
        tension_1[next_idx] = moments["tension_1"]
        tension_2[next_idx] = moments["tension_2"]
        target_torques[next_idx] = moments["target_torque"]
        motor_cable_torques[next_idx] = moments["motor_cable_torque"]
        rod_gravity_torques[next_idx] = moments["rod_gravity_torque"]

    detail_records[-1] = detail_records[-2].copy()
    detail_keys = sorted({key for record in detail_records for key in record})
    details = {
        key: np.array([record.get(key, np.nan) for record in detail_records], dtype=float)
        for key in detail_keys
    }

    return {
        "time": times,
        "state": states,
        "voltage": voltages,
        "reference": references,
        "cable_deformation": deformations,
        "cable_deformation_rate": deformation_rates,
        "delta_tension": delta_tensions,
        "tension_1": tension_1,
        "tension_2": tension_2,
        "target_torque": target_torques,
        "motor_cable_torque": motor_cable_torques,
        "rod_gravity_torque": rod_gravity_torques,
        "details": details,
    }
