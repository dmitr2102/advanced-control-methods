from __future__ import annotations

import math
from dataclasses import dataclass

from .base import ControllerOutput


@dataclass
class PositionPIDController:
    """PID controller that modulates carrier amplitude from the angle error."""

    carrier_frequency: float = 20.0
    base_amplitude: float = 86.5
    kp: float = 20.0
    ki: float = 3.8
    kd: float = 16.5
    min_amplitude: float = 62.5
    max_amplitude: float = 113.0
    phase: float = 0.0
    integral_error: float = 0.0
    previous_error: float = 0.0
    current_amplitude: float = 86.5
    last_time_value: float | None = None
    name: str = "pid_position"

    def reset(self) -> None:
        self.phase = 0.0
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.current_amplitude = self.base_amplitude
        self.last_time_value = None

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += self.carrier_frequency * dt

        error = abs(phi)
        if dt > 0.0:
            self.integral_error += error * dt
            derivative_error = (error - self.previous_error) / dt
        else:
            derivative_error = 0.0
        self.previous_error = error

        amplitude = self.base_amplitude
        amplitude += self.kp * error
        amplitude += self.ki * self.integral_error
        amplitude += self.kd * max(0.0, derivative_error)
        amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))
        self.current_amplitude = amplitude

        action = amplitude * math.cos(self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "carrier_frequency": self.carrier_frequency,
                "amplitude": amplitude,
                "kp": self.kp,
                "ki": self.ki,
                "kd": self.kd,
                "error": error,
                "phi_dot": phi_dot,
            },
        )


@dataclass
class CycleEnergyPIDController:
    """PID controller that regulates the averaged energy over each carrier cycle."""

    carrier_frequency: float = 14.0
    base_amplitude: float = 82.0
    kp: float = 21.8
    ki: float = 8.5
    kd: float = 6.6
    energy_weight: float = 6.3
    target_cycle_energy: float = 0.0
    min_amplitude: float = 68.5
    max_amplitude: float = 133.5
    phase: float = 0.0
    cycle_energy_sum: float = 0.0
    cycle_elapsed: float = 0.0
    cycle_error_integral: float = 0.0
    previous_cycle_error: float = 0.0
    last_time_value: float | None = None
    last_cycle_error: float = 0.0
    current_amplitude: float = 82.0
    name: str = "pid_cycle_energy"

    def reset(self) -> None:
        self.phase = 0.0
        self.cycle_energy_sum = 0.0
        self.cycle_elapsed = 0.0
        self.cycle_error_integral = 0.0
        self.previous_cycle_error = 0.0
        self.last_time_value = None
        self.last_cycle_error = 0.0
        self.current_amplitude = self.base_amplitude

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        phi, phi_dot = state
        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value
        self.phase += self.carrier_frequency * dt

        energy = 0.5 * phi_dot * phi_dot + self.energy_weight * (1.0 - math.cos(phi))
        self.cycle_energy_sum += energy * dt
        self.cycle_elapsed += dt

        cycle_period = 2.0 * math.pi / self.carrier_frequency
        if self.cycle_elapsed >= cycle_period and cycle_period > 0.0:
            average_cycle_energy = self.cycle_energy_sum / self.cycle_elapsed
            cycle_error = average_cycle_energy - self.target_cycle_energy
            self.cycle_error_integral += cycle_error * self.cycle_elapsed
            derivative_error = (cycle_error - self.previous_cycle_error) / self.cycle_elapsed
            self.previous_cycle_error = cycle_error
            self.last_cycle_error = cycle_error

            amplitude = self.base_amplitude
            amplitude += self.kp * cycle_error
            amplitude += self.ki * self.cycle_error_integral
            amplitude += self.kd * max(0.0, derivative_error)
            self.current_amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))

            self.cycle_energy_sum = 0.0
            self.cycle_elapsed = 0.0

        action = self.current_amplitude * math.cos(self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "carrier_frequency": self.carrier_frequency,
                "amplitude": self.current_amplitude,
                "kp": self.kp,
                "ki": self.ki,
                "kd": self.kd,
                "cycle_energy_error": self.last_cycle_error,
            },
        )
