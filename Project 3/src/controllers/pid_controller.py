from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .base import ControllerOutput


@dataclass
class PIDAngleController:
    """PID voltage controller for target pulley angle."""

    kp_voltage: float
    ki_voltage: float
    kd_voltage: float
    reference_amplitude: float
    reference_frequency: float
    max_voltage: float
    reference_mode: str = "sine"
    step_segment_duration: float = 5.0
    integrator_limit: float = 20.0
    derivative_filter_time: float = 0.02
    name: str = "pid_angle_voltage"
    integral_error: float = field(default=0.0, init=False)
    filtered_velocity_error: float = field(default=0.0, init=False)
    last_details: dict[str, float] = field(default_factory=dict, init=False)

    def reset(self) -> None:
        self.integral_error = 0.0
        self.filtered_velocity_error = 0.0
        self.last_details = {}

    def step_command(self, time_value: float) -> float:
        references = (0.0, 0.5 * np.pi, -0.5 * np.pi, 0.0)
        segment_idx = int(time_value // self.step_segment_duration)
        segment_idx = min(segment_idx, len(references) - 1)
        return float(references[segment_idx])

    def reference_angle(self, time_value: float) -> float:
        if self.reference_mode == "step":
            return self.step_command(time_value)
        return float(self.reference_amplitude * np.sin(self.reference_frequency * time_value))

    def reference_velocity(self, time_value: float) -> float:
        if self.reference_mode == "step":
            return 0.0
        return float(
            self.reference_amplitude
            * self.reference_frequency
            * np.cos(self.reference_frequency * time_value)
        )

    def reference_acceleration(self, time_value: float) -> float:
        if self.reference_mode == "step":
            return 0.0
        return float(
            -self.reference_amplitude
            * self.reference_frequency**2
            * np.sin(self.reference_frequency * time_value)
        )

    def update_derivative(self, velocity_error: float, dt: float) -> float:
        if self.derivative_filter_time <= 1.0e-12:
            self.filtered_velocity_error = velocity_error
            return velocity_error
        alpha = dt / (self.derivative_filter_time + dt)
        self.filtered_velocity_error = float(
            self.filtered_velocity_error
            + alpha * (velocity_error - self.filtered_velocity_error)
        )
        return self.filtered_velocity_error

    def compute_action(self, time_value: float, state: np.ndarray, dt: float) -> ControllerOutput:
        theta_l = float(state[0])
        omega_l = float(state[1])
        reference = self.reference_angle(time_value)
        reference_dot = self.reference_velocity(time_value)
        error = reference - theta_l
        velocity_error = reference_dot - omega_l
        derivative_signal = self.update_derivative(velocity_error, dt)

        previous_integral = self.integral_error
        self.integral_error = float(
            np.clip(
                self.integral_error + error * dt,
                -self.integrator_limit,
                self.integrator_limit,
            )
        )

        p_term = self.kp_voltage * error
        i_term = self.ki_voltage * self.integral_error
        d_term = self.kd_voltage * derivative_signal
        unsaturated_voltage = p_term + i_term + d_term
        voltage = float(np.clip(unsaturated_voltage, -self.max_voltage, self.max_voltage))

        saturation_pushes_same_direction = (
            abs(unsaturated_voltage) > self.max_voltage
            and np.sign(unsaturated_voltage) == np.sign(error)
        )
        if saturation_pushes_same_direction:
            self.integral_error = previous_integral
            i_term = self.ki_voltage * self.integral_error
            unsaturated_voltage = p_term + i_term + d_term
            voltage = float(np.clip(unsaturated_voltage, -self.max_voltage, self.max_voltage))

        self.last_details = {
            "reference": float(reference),
            "reference_velocity": float(reference_dot),
            "reference_acceleration": self.reference_acceleration(time_value),
            "theta_l": theta_l,
            "omega_l": omega_l,
            "angle_error": float(error),
            "velocity_error": float(velocity_error),
            "filtered_velocity_error": float(derivative_signal),
            "integral_error": float(self.integral_error),
            "p_term": float(p_term),
            "i_term": float(i_term),
            "d_term": float(d_term),
            "unsaturated_voltage": float(unsaturated_voltage),
            "voltage": float(voltage),
            "is_saturated": float(abs(unsaturated_voltage) > self.max_voltage),
        }
        return ControllerOutput(name=self.name, action=voltage, details=self.last_details.copy())
