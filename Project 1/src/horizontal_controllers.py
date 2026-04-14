from __future__ import annotations

import math
from dataclasses import dataclass

from controllers.base import ControllerOutput


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class HorizontalHarmonicController:
    gravity: float = 9.81
    length: float = 1.0
    amplitude: float = 800.0
    frequency: float = 50.0
    phase: float = 0.0
    name: str = "horizontal_harmonic"

    def reset(self) -> None:
        self.phase = 0.0

    @property
    def target_angle(self) -> float:
        cosine_value = 2.0 * self.gravity * self.length * (self.frequency ** 2) / (self.amplitude ** 2)
        cosine_value = min(0.999999, max(1e-6, cosine_value))
        return math.acos(cosine_value)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        del state
        action = self.amplitude * math.cos(self.frequency * time_value + self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "amplitude": self.amplitude,
                "frequency": self.frequency,
                "target_angle": self.target_angle,
            },
        )


@dataclass
class HorizontalLyapunovController:
    """Lyapunov-inspired controller for a stable side equilibrium.

    The controller uses the averaged side-equilibrium relation
        cos(theta*) = 2 g l omega^2 / alpha^2
    and shapes a desired equilibrium cosine as
        c_des = c_ref + k_e e + k_d theta_dot,
    where e = wrap(theta - theta_ref).
    """

    gravity: float = 9.81
    length: float = 1.0
    reference_amplitude: float = 800.0
    carrier_frequency: float = 50.0
    cosine_error_gain: float = 0.023505318845706027
    rate_gain: float = 0.19618509525626213
    min_cosine: float = 0.02999038143887106
    max_cosine: float = 0.4665915638167526
    phase: float = 0.0
    name: str = "horizontal_lyapunov"

    def reset(self) -> None:
        self.phase = 0.0

    @property
    def reference_cosine(self) -> float:
        return 2.0 * self.gravity * self.length * (self.carrier_frequency ** 2) / (self.reference_amplitude ** 2)

    @property
    def target_angle(self) -> float:
        cosine_value = min(0.999999, max(1e-6, self.reference_cosine))
        return math.acos(cosine_value)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        theta, theta_dot = state
        error = wrap_to_pi(theta - self.target_angle)
        cosine_desired = self.reference_cosine + self.cosine_error_gain * error + self.rate_gain * theta_dot
        cosine_desired = min(self.max_cosine, max(self.min_cosine, cosine_desired))
        amplitude = math.sqrt(2.0 * self.gravity * self.length * (self.carrier_frequency ** 2) / cosine_desired)
        action = amplitude * math.cos(self.carrier_frequency * time_value + self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "amplitude": amplitude,
                "carrier_frequency": self.carrier_frequency,
                "target_angle": self.target_angle,
                "error": error,
                "cosine_desired": cosine_desired,
            },
        )


@dataclass
class HorizontalAveragedEnergyController:
    """Averaged-energy controller for the horizontal excitation case.

    It uses the averaged side-equilibrium relation
        cos(theta*) = 2 g l omega^2 / alpha^2
    and increases the carrier amplitude as the averaged side-equilibrium energy grows.
    """

    gravity: float = 9.81
    length: float = 1.0
    reference_amplitude: float = 800.0
    carrier_frequency: float = 50.0
    angle_energy_gain: float = 4.0
    rate_energy_gain: float = 0.1
    min_cosine: float = 0.03
    max_cosine: float = 0.12
    phase: float = 0.0
    name: str = "horizontal_averaged_energy"

    def reset(self) -> None:
        self.phase = 0.0

    @property
    def reference_cosine(self) -> float:
        return 2.0 * self.gravity * self.length * (self.carrier_frequency ** 2) / (self.reference_amplitude ** 2)

    @property
    def target_angle(self) -> float:
        cosine_value = min(0.999999, max(1e-6, self.reference_cosine))
        return math.acos(cosine_value)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        theta, theta_dot = state
        error = wrap_to_pi(theta - self.target_angle)
        averaged_energy_proxy = (1.0 - math.cos(error)) + self.rate_energy_gain * (theta_dot ** 2)
        cosine_desired = self.reference_cosine / (1.0 + self.angle_energy_gain * averaged_energy_proxy)
        cosine_desired = min(self.max_cosine, max(self.min_cosine, cosine_desired))
        amplitude = math.sqrt(2.0 * self.gravity * self.length * (self.carrier_frequency ** 2) / cosine_desired)
        action = amplitude * math.cos(self.carrier_frequency * time_value + self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "amplitude": amplitude,
                "carrier_frequency": self.carrier_frequency,
                "target_angle": self.target_angle,
                "averaged_energy_proxy": averaged_energy_proxy,
                "cosine_desired": cosine_desired,
            },
        )


@dataclass
class HorizontalCycleEnergyPIDController:
    gravity: float = 9.81
    length: float = 1.0
    reference_amplitude: float = 800.0
    carrier_frequency: float = 50.0
    kp: float = 0.9824124315395419
    ki: float = 0.1965048611789378
    kd: float = 1.541676636373897
    energy_weight: float = 0.5209444559380662
    min_amplitude: float = 819.9699390479283
    max_amplitude: float = 950.7234897046674
    phase: float = 0.0
    cycle_energy_sum: float = 0.0
    cycle_elapsed: float = 0.0
    cycle_error_integral: float = 0.0
    previous_cycle_error: float = 0.0
    last_time_value: float | None = None
    current_amplitude: float = 800.0
    last_cycle_error: float = 0.0
    name: str = "horizontal_pid_cycle_energy"

    def reset(self) -> None:
        self.phase = 0.0
        self.cycle_energy_sum = 0.0
        self.cycle_elapsed = 0.0
        self.cycle_error_integral = 0.0
        self.previous_cycle_error = 0.0
        self.last_time_value = None
        self.current_amplitude = self.reference_amplitude
        self.last_cycle_error = 0.0

    @property
    def reference_cosine(self) -> float:
        return 2.0 * self.gravity * self.length * (self.carrier_frequency ** 2) / (self.reference_amplitude ** 2)

    @property
    def target_angle(self) -> float:
        cosine_value = min(0.999999, max(1e-6, self.reference_cosine))
        return math.acos(cosine_value)

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        theta, theta_dot = state
        if self.last_time_value is None:
            self.last_time_value = time_value
        dt = max(0.0, time_value - self.last_time_value)
        self.last_time_value = time_value

        error = wrap_to_pi(theta - self.target_angle)
        cycle_energy = 0.5 * theta_dot * theta_dot + self.energy_weight * (1.0 - math.cos(error))
        self.cycle_energy_sum += cycle_energy * dt
        self.cycle_elapsed += dt

        cycle_period = 2.0 * math.pi / self.carrier_frequency
        if self.cycle_elapsed >= cycle_period and cycle_period > 0.0:
            average_cycle_energy = self.cycle_energy_sum / self.cycle_elapsed
            cycle_error = average_cycle_energy
            self.cycle_error_integral += cycle_error * self.cycle_elapsed
            derivative_error = (cycle_error - self.previous_cycle_error) / self.cycle_elapsed
            self.previous_cycle_error = cycle_error
            self.last_cycle_error = cycle_error

            amplitude = self.reference_amplitude
            amplitude += self.kp * cycle_error
            amplitude += self.ki * self.cycle_error_integral
            amplitude += self.kd * max(0.0, derivative_error)
            self.current_amplitude = min(self.max_amplitude, max(self.min_amplitude, amplitude))

            self.cycle_energy_sum = 0.0
            self.cycle_elapsed = 0.0

        action = self.current_amplitude * math.cos(self.carrier_frequency * time_value + self.phase)
        return ControllerOutput(
            name=self.name,
            action=action,
            details={
                "amplitude": self.current_amplitude,
                "carrier_frequency": self.carrier_frequency,
                "target_angle": self.target_angle,
                "cycle_energy_error": self.last_cycle_error,
                "kp": self.kp,
                "ki": self.ki,
                "kd": self.kd,
            },
        )
