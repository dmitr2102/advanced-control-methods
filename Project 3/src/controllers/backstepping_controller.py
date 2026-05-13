from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from system import CablePulleyPlant

from .base import ControllerOutput


@dataclass
class BacksteppingController:
    """Dynamic-surface backstepping controller for the cable-pulley plant."""

    plant: CablePulleyPlant
    reference_amplitude: float
    reference_frequency: float
    reference_mode: str
    step_segment_duration: float
    shaper_frequency: float
    shaper_damping: float
    k1: float
    k2: float
    k3: float
    k4: float
    torque_filter_time: float
    current_filter_time: float
    max_torque_command: float
    max_current_command: float
    max_voltage: float
    relative_damping_gain: float = 0.0
    name: str = "dynamic_surface_backstepping"
    shaped_reference: float = field(default=0.0, init=False)
    shaped_reference_velocity: float = field(default=0.0, init=False)
    torque_command: float | None = field(default=None, init=False)
    current_command: float | None = field(default=None, init=False)
    last_details: dict[str, float] = field(default_factory=dict, init=False)

    def reset(self) -> None:
        self.shaped_reference = 0.0
        self.shaped_reference_velocity = 0.0
        self.torque_command = None
        self.current_command = None
        self.last_details = {}

    def step_command(self, time_value: float) -> float:
        references = (0.0, 0.5 * np.pi, -0.5 * np.pi, 0.0)
        segment_idx = int(time_value // self.step_segment_duration)
        segment_idx = min(segment_idx, len(references) - 1)
        return float(references[segment_idx])

    def raw_reference_angle(self, time_value: float) -> float:
        if self.reference_mode in {"step", "shaped_step"}:
            return self.step_command(time_value)
        return float(self.reference_amplitude * np.sin(self.reference_frequency * time_value))

    def reference_angle(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step":
            return float(self.shaped_reference)
        return self.raw_reference_angle(time_value)

    def reference_velocity(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step":
            return float(self.shaped_reference_velocity)
        if self.reference_mode == "step":
            return 0.0
        return float(
            self.reference_amplitude
            * self.reference_frequency
            * np.cos(self.reference_frequency * time_value)
        )

    def reference_acceleration(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step":
            command = self.step_command(time_value)
            return float(
                self.shaper_frequency**2 * (command - self.shaped_reference)
                - 2.0
                * self.shaper_damping
                * self.shaper_frequency
                * self.shaped_reference_velocity
            )
        if self.reference_mode == "step":
            return 0.0
        return float(
            -self.reference_amplitude
            * self.reference_frequency**2
            * np.sin(self.reference_frequency * time_value)
        )

    def cable_torque_drift_and_gain(self, state: np.ndarray, target_torque: float) -> tuple[float, float]:
        theta_l, omega_l, _, omega_m, _ = state
        p = self.plant.params
        q_dot = p.motor_radius * omega_m - p.target_radius * omega_l
        gravity_torque = self.plant.rod_gravity_torque(state)
        target_accel_without_current = (
            target_torque
            + gravity_torque
            - p.target_viscous_damping * omega_l
            - p.constant_load_torque
        ) / self.plant.target_total_inertia
        motor_accel_without_current = (
            -(p.motor_radius / p.target_radius) * target_torque
            - p.motor_viscous_damping * omega_m
        ) / self.plant.motor_total_inertia
        drift = 2.0 * p.target_radius * (
            p.cable_stiffness * q_dot
            + p.cable_damping
            * (
                p.motor_radius * motor_accel_without_current
                - p.target_radius * target_accel_without_current
            )
        )
        gain = (
            2.0
            * p.target_radius
            * p.cable_damping
            * p.motor_radius
            * p.torque_constant
            / self.plant.motor_total_inertia
        )
        del theta_l
        return float(drift), float(gain)

    def compute_action(self, time_value: float, state: np.ndarray, dt: float) -> ControllerOutput:
        raw_reference = self.raw_reference_angle(time_value)
        theta_l, omega_l, _, _, current = state
        p = self.plant.params
        moments = self.plant.moments(state)
        target_torque = moments["target_torque"]
        gravity_torque = moments["rod_gravity_torque"]
        _, q_dot = self.plant.cable_kinematics(state)

        reference = self.reference_angle(time_value)
        reference_dot = self.reference_velocity(time_value)
        reference_ddot = self.reference_acceleration(time_value)

        z1 = theta_l - reference
        alpha1 = reference_dot - self.k1 * z1
        z2 = omega_l - alpha1
        alpha1_dot = reference_ddot - self.k1 * (omega_l - reference_dot)

        torque_desired = (
            self.plant.target_total_inertia * (alpha1_dot - z1 - self.k2 * z2)
            - gravity_torque
            + p.target_viscous_damping * omega_l
            + p.constant_load_torque
        )
        relative_damping_torque = -self.relative_damping_gain * q_dot
        torque_desired += relative_damping_torque
        torque_desired = float(
            np.clip(torque_desired, -self.max_torque_command, self.max_torque_command)
        )

        if self.torque_command is None:
            self.torque_command = float(target_torque)
        if self.current_command is None:
            self.current_command = float(current)

        torque_command_dot = (torque_desired - self.torque_command) / self.torque_filter_time
        torque_command = self.torque_command
        z3 = target_torque - torque_command

        torque_drift, torque_gain = self.cable_torque_drift_and_gain(state, target_torque)
        if abs(torque_gain) < 1.0e-9:
            raise ValueError("Backstepping current gain is too small; cable damping must be positive.")

        current_desired = (
            -torque_drift
            + torque_command_dot
            - z2 / self.plant.target_total_inertia
            - self.k3 * z3
        ) / torque_gain
        current_desired = float(
            np.clip(current_desired, -self.max_current_command, self.max_current_command)
        )

        current_command_dot = (current_desired - self.current_command) / self.current_filter_time
        current_command = self.current_command
        z4 = current - current_command

        voltage = (
            p.motor_resistance * current
            + p.back_emf_constant * state[3]
            + p.motor_inductance
            * (
                current_command_dot
                - torque_gain * z3
                - self.k4 * z4
            )
        )
        voltage = float(np.clip(voltage, -self.max_voltage, self.max_voltage))

        self.torque_command = float(self.torque_command + dt * torque_command_dot)
        self.current_command = float(self.current_command + dt * current_command_dot)
        if self.reference_mode == "shaped_step":
            reference_ddot = self.reference_acceleration(time_value)
            self.shaped_reference = float(self.shaped_reference + dt * self.shaped_reference_velocity)
            self.shaped_reference_velocity = float(
                self.shaped_reference_velocity + dt * reference_ddot
            )

        lyapunov = 0.5 * (z1**2 + z2**2 + z3**2 + z4**2)
        self.last_details = {
            "reference": float(reference),
            "raw_reference": float(raw_reference),
            "reference_velocity": float(reference_dot),
            "reference_acceleration": float(reference_ddot),
            "z1": float(z1),
            "z2": float(z2),
            "z3": float(z3),
            "z4": float(z4),
            "alpha1": float(alpha1),
            "alpha1_dot": float(alpha1_dot),
            "torque_desired": float(torque_desired),
            "torque_command": float(torque_command),
            "torque_command_dot": float(torque_command_dot),
            "relative_velocity": float(q_dot),
            "relative_damping_torque": float(relative_damping_torque),
            "torque_drift": float(torque_drift),
            "torque_gain": float(torque_gain),
            "current_desired": float(current_desired),
            "current_command": float(current_command),
            "current_command_dot": float(current_command_dot),
            "lyapunov": float(lyapunov),
            "voltage": float(voltage),
        }
        return ControllerOutput(name=self.name, action=voltage, details=self.last_details.copy())
