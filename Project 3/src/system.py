from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CablePulleyPlantParameters:
    target_pulley_inertia: float = 0.00012
    motor_pulley_inertia: float = 0.00008
    motor_rotor_inertia: float = 0.00018
    target_radius: float = 0.04
    motor_radius: float = 0.04
    cable_stiffness: float = 3500.0
    cable_damping: float = 18.0
    cable_pretension: float = 40.0
    target_viscous_damping: float = 0.012
    motor_viscous_damping: float = 0.004
    constant_load_torque: float = 0.0
    rod_mass: float = 0.25
    rod_length: float = 0.35
    gravity: float = 9.81
    motor_resistance: float = 1.2
    motor_inductance: float = 0.0025
    torque_constant: float = 0.055
    back_emf_constant: float = 0.055
    max_voltage: float = 24.0


class CablePulleyPlant:
    """Elastic two-cable pulley transmission driven by a DC motor.

    State notation:
        s = [theta_l, omega_l, theta_m, omega_m, i]^T

    where theta_l is the target/load pulley angle, theta_m is the motor pulley
    angle, and i is the DC motor armature current. The action a is motor voltage.
    """

    def __init__(self, params: CablePulleyPlantParameters) -> None:
        self.params = params

    @property
    def rod_inertia(self) -> float:
        p = self.params
        return p.rod_mass * p.rod_length**2 / 3.0

    @property
    def target_total_inertia(self) -> float:
        return self.params.target_pulley_inertia + self.rod_inertia

    @property
    def motor_total_inertia(self) -> float:
        p = self.params
        return p.motor_rotor_inertia + p.motor_pulley_inertia

    def cable_kinematics(self, state: np.ndarray) -> tuple[float, float]:
        theta_l, omega_l, theta_m, omega_m, _ = state
        p = self.params
        deformation = p.motor_radius * theta_m - p.target_radius * theta_l
        deformation_rate = p.motor_radius * omega_m - p.target_radius * omega_l
        return float(deformation), float(deformation_rate)

    def cable_tension_difference(self, state: np.ndarray) -> float:
        deformation, deformation_rate = self.cable_kinematics(state)
        p = self.params
        return 2.0 * (p.cable_stiffness * deformation + p.cable_damping * deformation_rate)

    def cable_tensions(self, state: np.ndarray) -> tuple[float, float]:
        deformation, deformation_rate = self.cable_kinematics(state)
        p = self.params
        tension_offset = p.cable_stiffness * deformation + p.cable_damping * deformation_rate
        return (
            float(p.cable_pretension + tension_offset),
            float(p.cable_pretension - tension_offset),
        )

    def rod_gravity_torque(self, state: np.ndarray) -> float:
        theta_l = float(state[0])
        p = self.params
        center_of_mass = 0.5 * p.rod_length
        return float(-p.rod_mass * p.gravity * center_of_mass * np.sin(theta_l))

    def moments(self, state: np.ndarray) -> dict[str, float]:
        delta_tension = self.cable_tension_difference(state)
        tension_1, tension_2 = self.cable_tensions(state)
        gravity_torque = self.rod_gravity_torque(state)
        p = self.params
        target_torque = p.target_radius * delta_tension
        motor_cable_torque = -p.motor_radius * delta_tension
        return {
            "delta_tension": float(delta_tension),
            "tension_1": float(tension_1),
            "tension_2": float(tension_2),
            "target_torque": float(target_torque),
            "motor_cable_torque": float(motor_cable_torque),
            "rod_gravity_torque": float(gravity_torque),
        }

    def derivative(self, time_value: float, state: np.ndarray, voltage: float) -> np.ndarray:
        del time_value
        theta_l, omega_l, theta_m, omega_m, current = state
        del theta_l, theta_m
        p = self.params
        voltage = float(np.clip(voltage, -p.max_voltage, p.max_voltage))
        moments = self.moments(state)

        theta_l_dot = omega_l
        omega_l_dot = (
            moments["target_torque"]
            + moments["rod_gravity_torque"]
            - p.target_viscous_damping * omega_l
            - p.constant_load_torque
        ) / self.target_total_inertia
        theta_m_dot = omega_m
        omega_m_dot = (
            p.torque_constant * current
            + moments["motor_cable_torque"]
            - p.motor_viscous_damping * omega_m
        ) / self.motor_total_inertia
        current_dot = (
            voltage
            - p.motor_resistance * current
            - p.back_emf_constant * omega_m
        ) / p.motor_inductance

        return np.array(
            [theta_l_dot, omega_l_dot, theta_m_dot, omega_m_dot, current_dot],
            dtype=float,
        )
