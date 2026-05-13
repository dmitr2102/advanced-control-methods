from __future__ import annotations

import copy
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
THIS_DIR = PROJECT_ROOT / "backstep_explicit_derivatives"
for path in (SRC_ROOT, SCRIPTS_ROOT, THIS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import matplotlib

matplotlib.use("Agg")

import numpy as np

from config import load_json
from controllers.base import ControllerOutput
from animation_utils import animate_pulleys
from run_explicit_derivatives_backstepping import ExplicitDerivativeBacksteppingController
from shaped_step_utils import plot_shaped_step_history, shaped_step_summary
from simulation import SimulationConfig, simulate
from system import CablePulleyPlant, CablePulleyPlantParameters


OUTPUT_ROOT = PROJECT_ROOT / "backstep_explicit_derivatives" / "shaped_step4_case"


@dataclass
class FourthOrderShapedStepBacksteppingController(ExplicitDerivativeBacksteppingController):
    """Explicit backstepping with a fourth-order shaped step reference.

    The shaper is

        (D^2 + 2*zeta*omega_s*D + omega_s^2)^2 r = omega_s^4 theta_cmd.

    For a raw step command this makes r, dot(r), ddot(r), and r^(3)
    continuous. The snap r^(4) is finite and piecewise smooth.
    """

    shaped_reference_acceleration: float = field(default=0.0, init=False)
    shaped_reference_jerk: float = field(default=0.0, init=False)

    def reset(self) -> None:
        super().reset()
        self.shaped_reference_acceleration = 0.0
        self.shaped_reference_jerk = 0.0

    def raw_reference_angle(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step_4th_order":
            return self.step_command(time_value)
        return super().raw_reference_angle(time_value)

    def reference_angle(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step_4th_order":
            return float(self.shaped_reference)
        return super().reference_angle(time_value)

    def reference_velocity(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step_4th_order":
            return float(self.shaped_reference_velocity)
        return super().reference_velocity(time_value)

    def reference_acceleration(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step_4th_order":
            return float(self.shaped_reference_acceleration)
        return super().reference_acceleration(time_value)

    def reference_jerk(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step_4th_order":
            return float(self.shaped_reference_jerk)
        return super().reference_jerk(time_value)

    def reference_snap(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step_4th_order":
            return self.shaper_snap(time_value)
        return super().reference_snap(time_value)

    def shaper_snap(self, time_value: float) -> float:
        command = self.step_command(time_value)
        omega = self.shaper_frequency
        zeta = self.shaper_damping
        return float(
            omega**4 * (command - self.shaped_reference)
            - 4.0 * zeta * omega * self.shaped_reference_jerk
            - (4.0 * zeta**2 + 2.0) * omega**2 * self.shaped_reference_acceleration
            - 4.0 * zeta * omega**3 * self.shaped_reference_velocity
        )

    def shaper_dynamics(self, time_value: float, shaper_state: np.ndarray) -> np.ndarray:
        position, velocity, acceleration, jerk = shaper_state
        command = self.step_command(time_value)
        omega = self.shaper_frequency
        zeta = self.shaper_damping
        snap = (
            omega**4 * (command - position)
            - 4.0 * zeta * omega * jerk
            - (4.0 * zeta**2 + 2.0) * omega**2 * acceleration
            - 4.0 * zeta * omega**3 * velocity
        )
        return np.array([velocity, acceleration, jerk, snap], dtype=float)

    def advance_shaper(self, time_value: float, dt: float) -> None:
        shaper_state = np.array(
            [
                self.shaped_reference,
                self.shaped_reference_velocity,
                self.shaped_reference_acceleration,
                self.shaped_reference_jerk,
            ],
            dtype=float,
        )
        k1 = self.shaper_dynamics(time_value, shaper_state)
        k2 = self.shaper_dynamics(time_value + 0.5 * dt, shaper_state + 0.5 * dt * k1)
        k3 = self.shaper_dynamics(time_value + 0.5 * dt, shaper_state + 0.5 * dt * k2)
        k4 = self.shaper_dynamics(time_value + dt, shaper_state + dt * k3)
        shaper_state = shaper_state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0

        self.shaped_reference = float(shaper_state[0])
        self.shaped_reference_velocity = float(shaper_state[1])
        self.shaped_reference_acceleration = float(shaper_state[2])
        self.shaped_reference_jerk = float(shaper_state[3])

    def compute_action(self, time_value: float, state: np.ndarray, dt: float) -> ControllerOutput:
        output = super().compute_action(time_value, state, dt)
        if self.reference_mode == "shaped_step_4th_order":
            details = output.details
            details["reference_jerk"] = float(self.reference_jerk(time_value))
            details["reference_snap"] = float(self.reference_snap(time_value))
            details["shaper_order"] = 4.0
            self.last_details.update(details)
            self.advance_shaper(time_value, dt)
        return output


def build_fourth_order_shaper_controller(
    config: dict,
    limit_mode: str,
    gain_scale: float = 1.0,
) -> tuple[CablePulleyPlant, FourthOrderShapedStepBacksteppingController, SimulationConfig]:
    plant_cfg = config["plant"]
    sim_cfg = config["simulation"]
    reference_cfg = config["controller"]
    controller_cfg = config["backstepping_controller"]
    initial_cfg = config["initial_state"]

    plant = CablePulleyPlant(CablePulleyPlantParameters(**plant_cfg))
    controller = FourthOrderShapedStepBacksteppingController(
        plant=plant,
        reference_amplitude=float(reference_cfg["reference_amplitude"]),
        reference_frequency=float(reference_cfg["reference_frequency"]),
        reference_mode="shaped_step_4th_order",
        step_segment_duration=float(controller_cfg.get("step_segment_duration", 5.0)),
        shaper_frequency=float(controller_cfg.get("shaper_frequency", 2.0)),
        shaper_damping=float(controller_cfg.get("shaper_damping", 1.0)),
        k1=float(controller_cfg["k1"]) * gain_scale,
        k2=float(controller_cfg["k2"]) * gain_scale,
        k3=float(controller_cfg["k3"]) * gain_scale,
        k4=float(controller_cfg["k4"]) * gain_scale,
        max_torque_command=float(controller_cfg["max_torque_command"]),
        max_current_command=float(controller_cfg["max_current_command"]),
        max_voltage=float(controller_cfg["max_voltage"]),
        limit_mode=limit_mode,
    )
    sim_config = SimulationConfig(
        dt=float(sim_cfg["dt"]),
        horizon=float(sim_cfg["horizon"]),
        initial_state=np.array(
            [
                initial_cfg["theta_l"],
                initial_cfg["omega_l"],
                initial_cfg["theta_m"],
                initial_cfg["omega_m"],
                initial_cfg["current"],
            ],
            dtype=float,
        ),
    )
    return plant, controller, sim_config


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    base_config = load_json(PROJECT_ROOT / "configs" / "default.json")
    config = copy.deepcopy(base_config)
    config["backstepping_controller"]["relative_damping_gain"] = 0.0
    max_voltage = float(config["backstepping_controller"]["max_voltage"])

    cases = [
        ("shaped_step4_w2_hard_same_gains", "hard", 1.0, 2.0),
        ("shaped_step4_w4_hard_same_gains", "hard", 1.0, 4.0),
        ("shaped_step4_w6_hard_same_gains", "hard", 1.0, 6.0),
        ("shaped_step4_w8_hard_half_gains", "hard", 0.5, 8.0),
        ("shaped_step4_w8_hard_same_gains", "hard", 1.0, 8.0),
        ("shaped_step4_w8_no_virtual_limits", "none", 1.0, 8.0),
    ]

    results: dict[str, dict[str, np.ndarray]] = {}
    summaries: list[dict[str, float | str]] = []
    for name, limit_mode, gain_scale, shaper_frequency in cases:
        case_config = copy.deepcopy(config)
        case_config["backstepping_controller"]["shaper_frequency"] = shaper_frequency
        plant, controller, sim_config = build_fourth_order_shaper_controller(
            case_config,
            limit_mode,
            gain_scale,
        )
        result = simulate(plant, controller, sim_config)
        results[name] = result
        metrics = shaped_step_summary(name, result, max_voltage)
        metrics["shaper_frequency_rad_s"] = float(shaper_frequency)
        metrics["gain_scale"] = float(gain_scale)
        summaries.append(metrics)

    selected = min(summaries, key=lambda item: item["score"])
    selected_name = str(selected["name"])
    selected_gain_scale = float(selected["gain_scale"])
    controller_cfg = config["backstepping_controller"]
    selected_gains = {
        "k1": float(controller_cfg["k1"]) * selected_gain_scale,
        "k2": float(controller_cfg["k2"]) * selected_gain_scale,
        "k3": float(controller_cfg["k3"]) * selected_gain_scale,
        "k4": float(controller_cfg["k4"]) * selected_gain_scale,
    }

    plot_shaped_step_history(
        results[selected_name],
        OUTPUT_ROOT / "selected_shaped_step4_state_history.png",
        "Explicit backstepping fourth-order shaped step",
    )
    animate_pulleys(
        results[selected_name],
        OUTPUT_ROOT / "selected_shaped_step4_animation.gif",
        fps=12,
        title="Fourth-Order Shaped Backstepping",
        annotation_lines=[
            f"k1={selected_gains['k1']:.1f}, k2={selected_gains['k2']:.1f}",
            f"k3={selected_gains['k3']:.1f}, k4={selected_gains['k4']:.1f}",
        ],
    )

    ranked = sorted(summaries, key=lambda item: item["score"])
    zeta = float(config["backstepping_controller"]["shaper_damping"])
    payload = {
        "shaper": {
            "type": "fourth_order_repeated_second_order",
            "equation": "(D^2 + 2*zeta*omega_s*D + omega_s^2)^2 r = omega_s^4 theta_cmd",
            "frequency_rad_s_sweep": [2.0, 4.0, 6.0, 8.0],
            "damping_ratio": zeta,
        },
        "selected": selected_name,
        "summaries": ranked,
    }
    (OUTPUT_ROOT / "shaped_step4_summary.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
