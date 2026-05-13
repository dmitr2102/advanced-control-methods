from __future__ import annotations

import copy
import json
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
NO_FILTER_ROOT = PROJECT_ROOT / "log" / "backstep_no_filter"
for path in (SRC_ROOT, SCRIPTS_ROOT, NO_FILTER_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from config import build_backstepping_from_config, load_json
from animation_utils import animate_pulleys
from run_no_filter_backstepping import (
    NoFilterBacksteppingController,
    build_no_filter_controller,
    plot_comparison,
    plot_no_filter_history,
    score,
    summarize,
)
from simulation import SimulationConfig, simulate
from system import CablePulleyPlant, CablePulleyPlantParameters


OUTPUT_ROOT = PROJECT_ROOT / "backstep_explicit_derivatives"
ARCHIVE_OUTPUT_ROOT = PROJECT_ROOT / "log" / "backstep_explicit_derivatives"


@dataclass
class ExplicitDerivativeBacksteppingController(NoFilterBacksteppingController):
    """Original no-filter backstepping with explicit derivative formulas."""

    name: str = "explicit_derivative_backstepping"

    def reference_jerk(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step":
            reference_ddot = self.reference_acceleration(time_value)
            return float(
                -self.shaper_frequency**2 * self.shaped_reference_velocity
                - 2.0 * self.shaper_damping * self.shaper_frequency * reference_ddot
            )
        if self.reference_mode == "step":
            return 0.0
        return float(
            -self.reference_amplitude
            * self.reference_frequency**3
            * np.cos(self.reference_frequency * time_value)
        )

    def reference_snap(self, time_value: float) -> float:
        if self.reference_mode == "shaped_step":
            reference_ddot = self.reference_acceleration(time_value)
            reference_jerk = self.reference_jerk(time_value)
            return float(
                -self.shaper_frequency**2 * reference_ddot
                - 2.0 * self.shaper_damping * self.shaper_frequency * reference_jerk
            )
        if self.reference_mode == "step":
            return 0.0
        return float(
            self.reference_amplitude
            * self.reference_frequency**4
            * np.sin(self.reference_frequency * time_value)
        )

    def limit_with_second_derivative(
        self,
        raw_value: float,
        raw_derivative: float,
        raw_second_derivative: float,
        limit: float,
    ) -> tuple[float, float, float]:
        if self.limit_mode == "none":
            return float(raw_value), float(raw_derivative), float(raw_second_derivative)
        if self.limit_mode == "smooth":
            ratio = raw_value / limit
            ratio = float(np.clip(ratio, -50.0, 50.0))
            tanh_ratio = float(np.tanh(ratio))
            sech2 = float(1.0 / np.cosh(ratio) ** 2)
            limited = limit * tanh_ratio
            limited_dot = sech2 * raw_derivative
            limited_ddot = (
                sech2 * raw_second_derivative
                - 2.0 * tanh_ratio * sech2 * raw_derivative**2 / limit
            )
            return float(limited), float(limited_dot), float(limited_ddot)
        if abs(raw_value) < limit:
            return float(raw_value), float(raw_derivative), float(raw_second_derivative)
        return float(np.clip(raw_value, -limit, limit)), 0.0, 0.0

    def target_acceleration(self, state: np.ndarray) -> float:
        _, omega_l, _, _, _ = state
        p = self.plant.params
        moments = self.plant.moments(state)
        return float(
            (
                moments["target_torque"]
                + moments["rod_gravity_torque"]
                - p.target_viscous_damping * omega_l
                - p.constant_load_torque
            )
            / self.plant.target_total_inertia
        )

    def cable_torque_drift_gain_and_derivative(
        self,
        state: np.ndarray,
    ) -> tuple[float, float, float, float]:
        theta_l, omega_l, _, omega_m, current = state
        p = self.plant.params
        moments = self.plant.moments(state)
        target_torque = moments["target_torque"]
        gravity_torque = moments["rod_gravity_torque"]
        q_dot = p.motor_radius * omega_m - p.target_radius * omega_l

        target_accel = (
            target_torque
            + gravity_torque
            - p.target_viscous_damping * omega_l
            - p.constant_load_torque
        ) / self.plant.target_total_inertia
        motor_accel_without_current = (
            -(p.motor_radius / p.target_radius) * target_torque
            - p.motor_viscous_damping * omega_m
        ) / self.plant.motor_total_inertia
        motor_accel = motor_accel_without_current + (
            p.torque_constant * current / self.plant.motor_total_inertia
        )

        drift = 2.0 * p.target_radius * (
            p.cable_stiffness * q_dot
            + p.cable_damping
            * (
                p.motor_radius * motor_accel_without_current
                - p.target_radius * target_accel
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
        target_torque_dot = drift + gain * current

        gravity_torque_dot = (
            -p.rod_mass
            * p.gravity
            * 0.5
            * p.rod_length
            * np.cos(theta_l)
            * omega_l
        )
        target_accel_dot = (
            target_torque_dot
            + gravity_torque_dot
            - p.target_viscous_damping * target_accel
        ) / self.plant.target_total_inertia
        motor_accel_without_current_dot = (
            -(p.motor_radius / p.target_radius) * target_torque_dot
            - p.motor_viscous_damping * motor_accel
        ) / self.plant.motor_total_inertia
        q_ddot = p.motor_radius * motor_accel - p.target_radius * target_accel
        drift_dot = 2.0 * p.target_radius * (
            p.cable_stiffness * q_ddot
            + p.cable_damping
            * (
                p.motor_radius * motor_accel_without_current_dot
                - p.target_radius * target_accel_dot
            )
        )

        return (
            float(drift),
            float(gain),
            float(drift_dot),
            float(target_torque_dot),
        )

    def torque_desired_derivatives(
        self,
        time_value: float,
        state: np.ndarray,
    ) -> tuple[float, float, float, float, float, float]:
        theta_l, omega_l, _, _, _ = state
        p = self.plant.params
        target_accel = self.target_acceleration(state)
        drift, gain, _, _ = self.cable_torque_drift_gain_and_derivative(state)
        target_torque_dot = drift + gain * state[4]

        gravity_torque = self.plant.rod_gravity_torque(state)
        gravity_torque_dot = (
            -p.rod_mass
            * p.gravity
            * 0.5
            * p.rod_length
            * np.cos(theta_l)
            * omega_l
        )
        gravity_torque_ddot = (
            p.rod_mass
            * p.gravity
            * 0.5
            * p.rod_length
            * np.sin(theta_l)
            * omega_l**2
            - p.rod_mass
            * p.gravity
            * 0.5
            * p.rod_length
            * np.cos(theta_l)
            * target_accel
        )
        target_accel_dot = (
            target_torque_dot
            + gravity_torque_dot
            - p.target_viscous_damping * target_accel
        ) / self.plant.target_total_inertia

        reference = self.reference_angle(time_value)
        reference_dot = self.reference_velocity(time_value)
        reference_ddot = self.reference_acceleration(time_value)
        reference_jerk = self.reference_jerk(time_value)
        reference_snap = self.reference_snap(time_value)

        z1 = theta_l - reference
        z1_dot = omega_l - reference_dot
        z1_ddot = target_accel - reference_ddot
        z1_dddot = target_accel_dot - reference_jerk

        alpha1 = reference_dot - self.k1 * z1
        alpha1_dot = reference_ddot - self.k1 * z1_dot
        alpha1_ddot = reference_jerk - self.k1 * z1_ddot
        alpha1_dddot = reference_snap - self.k1 * z1_dddot

        z2 = omega_l - alpha1
        z2_dot = target_accel - alpha1_dot
        z2_ddot = target_accel_dot - alpha1_ddot

        raw_torque = (
            self.plant.target_total_inertia
            * (alpha1_dot - z1 - self.k2 * z2)
            - gravity_torque
            + p.target_viscous_damping * omega_l
            + p.constant_load_torque
        )
        raw_torque_dot = (
            self.plant.target_total_inertia
            * (alpha1_ddot - z1_dot - self.k2 * z2_dot)
            - gravity_torque_dot
            + p.target_viscous_damping * target_accel
        )
        raw_torque_ddot = (
            self.plant.target_total_inertia
            * (alpha1_dddot - z1_ddot - self.k2 * z2_ddot)
            - gravity_torque_ddot
            + p.target_viscous_damping * target_accel_dot
        )
        torque, torque_dot, torque_ddot = self.limit_with_second_derivative(
            raw_torque,
            raw_torque_dot,
            raw_torque_ddot,
            self.max_torque_command,
        )
        return (
            float(torque),
            float(torque_dot),
            float(torque_ddot),
            float(raw_torque),
            float(raw_torque_dot),
            float(raw_torque_ddot),
        )

    def torque_desired_and_derivative(
        self,
        time_value: float,
        state: np.ndarray,
    ) -> tuple[float, float, float, float]:
        torque, torque_dot, _, raw_torque, raw_torque_dot, _ = (
            self.torque_desired_derivatives(time_value, state)
        )
        return torque, torque_dot, raw_torque, raw_torque_dot

    def current_desired_and_derivative(
        self,
        time_value: float,
        state: np.ndarray,
    ) -> tuple[float, float, float, float]:
        moments = self.plant.moments(state)
        target_torque = moments["target_torque"]
        torque_desired, torque_desired_dot, torque_desired_ddot, _, _, _ = (
            self.torque_desired_derivatives(time_value, state)
        )
        _, z2, _, alpha1_dot = self.error_terms(time_value, state)
        z3 = target_torque - torque_desired

        target_accel = self.target_acceleration(state)
        z2_dot = target_accel - alpha1_dot
        torque_drift, torque_gain, torque_drift_dot, target_torque_dot = (
            self.cable_torque_drift_gain_and_derivative(state)
        )
        if abs(torque_gain) < 1.0e-9:
            raise ValueError("Backstepping current gain is too small; cable damping must be positive.")

        z3_dot = target_torque_dot - torque_desired_dot
        raw_current = (
            -torque_drift
            + torque_desired_dot
            - z2 / self.plant.target_total_inertia
            - self.k3 * z3
        ) / torque_gain
        raw_current_dot = (
            -torque_drift_dot
            + torque_desired_ddot
            - z2_dot / self.plant.target_total_inertia
            - self.k3 * z3_dot
        ) / torque_gain
        current, current_dot = self.limit_with_derivative(
            raw_current,
            raw_current_dot,
            self.max_current_command,
        )
        return float(current), float(current_dot), float(raw_current), float(raw_current_dot)

    def current_desired(self, time_value: float, state: np.ndarray) -> float:
        current, _, _, _ = self.current_desired_and_derivative(time_value, state)
        return current

    def current_desired_dot(self, time_value: float, state: np.ndarray) -> float:
        _, current_dot, _, _ = self.current_desired_and_derivative(time_value, state)
        return current_dot


def build_explicit_controller(
    config: dict,
    limit_mode: str,
    gain_scale: float = 1.0,
) -> tuple[CablePulleyPlant, ExplicitDerivativeBacksteppingController, SimulationConfig]:
    plant_cfg = config["plant"]
    sim_cfg = config["simulation"]
    reference_cfg = config["controller"]
    controller_cfg = config["backstepping_controller"]
    initial_cfg = config["initial_state"]

    plant = CablePulleyPlant(CablePulleyPlantParameters(**plant_cfg))
    controller = ExplicitDerivativeBacksteppingController(
        plant=plant,
        reference_amplitude=float(reference_cfg["reference_amplitude"]),
        reference_frequency=float(reference_cfg["reference_frequency"]),
        reference_mode=str(controller_cfg.get("reference_mode", "sine")),
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


def plot_derivative_check(
    explicit_result: dict[str, np.ndarray],
    directional_result: dict[str, np.ndarray],
    output_path: Path,
) -> None:
    time = explicit_result["time"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    pairs = [
        ("theta error", explicit_result["reference"] - explicit_result["state"][:, 0],
         directional_result["reference"] - directional_result["state"][:, 0]),
        ("torque desired", explicit_result["details"]["torque_desired"],
         directional_result["details"]["torque_desired"]),
        ("current desired", explicit_result["details"]["current_desired"],
         directional_result["details"]["current_desired"]),
        ("voltage", explicit_result["voltage"], directional_result["voltage"]),
    ]
    for axis, (label, explicit_signal, directional_signal) in zip(axes, pairs):
        axis.plot(time, explicit_signal, linewidth=1.1, label="explicit")
        axis.plot(time, directional_signal, "--", linewidth=1.0, label="directional")
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Explicit derivative check against directional derivative", y=0.995)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_readme(
    summaries: list[dict[str, float | str]],
    selected_name: str,
) -> None:
    lines = [
        "# Explicit-Derivative Backstepping",
        "",
        "This folder tests the original backstepping controller without dynamic-surface filters and without additional elastic-mode damping.",
        "",
        "Unlike `backstep_no_filter`, this version computes `dot(i_d)` explicitly:",
        "",
        "```math",
        "\\dot i_d = \\frac{-\\dot F_c + \\ddot\\tau_c^d - \\dot z_2/J_l-k_3\\dot z_3}{G_c}.",
        "```",
        "",
        "The auxiliary derivatives are:",
        "",
        "```math",
        "\\dot z_3 = \\dot\\tau_c - \\dot\\tau_c^d, \\qquad \\dot\\tau_c=F_c(s)+G_ci.",
        "```",
        "",
        "No terms of the form `-k_q q` or `-d_q dot(q)` are added to the desired torque.",
        "",
        "## Metrics",
        "",
        "| Case | Score | RMS error [rad] | HP RMS qdot [m/s] | RMS dV/dt [V/s] | Max raw V [V] | Max current [A] |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for item in summaries:
        lines.append(
            f"| {item['name']} | {item['score']:.4f} | "
            f"{item['rms_angle_error_rad']:.4f} | "
            f"{item['rms_highpass_relative_velocity_m_s']:.4f} | "
            f"{item['rms_voltage_rate_V_s']:.1f} | "
            f"{item['max_abs_unsaturated_voltage_V']:.1f} | "
            f"{item['max_abs_current_A']:.2f} |"
        )
    lines.extend(
        [
            "",
            f"Selected case for plotting: `{selected_name}`.",
            "",
            "## Files",
            "",
            "- `comparison.png`: filtered baseline vs selected explicit-derivative case.",
            "- `derivative_check.png`: explicit derivative case vs previous directional-derivative no-filter case.",
            "- `selected_explicit_state_history.png`: detailed state plot.",
            "- `selected_explicit_animation.gif`: animation for the selected explicit case.",
            "- `summary.json`: full metrics.",
            "",
        ]
    )
    ARCHIVE_OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    (ARCHIVE_OUTPUT_ROOT / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    ARCHIVE_OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    base_config = load_json(PROJECT_ROOT / "configs" / "default.json")
    config = copy.deepcopy(base_config)
    config["backstepping_controller"]["reference_mode"] = "sine"
    config["backstepping_controller"]["relative_damping_gain"] = 0.0
    max_voltage = float(config["backstepping_controller"]["max_voltage"])

    baseline_plant, baseline_controller, baseline_sim = build_backstepping_from_config(config)
    baseline_result = simulate(baseline_plant, baseline_controller, baseline_sim)
    baseline_summary = summarize("filtered_baseline", baseline_result, max_voltage)
    baseline_summary["score"] = score(baseline_summary)

    directional_plant, directional_controller, directional_sim = build_no_filter_controller(
        config,
        "hard",
        0.5,
    )
    directional_result = simulate(directional_plant, directional_controller, directional_sim)
    directional_summary = summarize(
        "directional_no_filter_hard_half_gains",
        directional_result,
        max_voltage,
    )
    directional_summary["score"] = score(directional_summary)

    cases = [
        ("explicit_hard_half_gains", "hard", 0.5),
        ("explicit_hard_same_gains", "hard", 1.0),
        ("explicit_no_virtual_limits", "none", 1.0),
    ]
    results: dict[str, dict[str, np.ndarray]] = {
        "filtered_baseline": baseline_result,
        "directional_no_filter_hard_half_gains": directional_result,
    }
    summaries: list[dict[str, float | str]] = [baseline_summary, directional_summary]

    for name, limit_mode, gain_scale in cases:
        plant, controller, sim_config = build_explicit_controller(config, limit_mode, gain_scale)
        result = simulate(plant, controller, sim_config)
        results[name] = result
        metrics = summarize(name, result, max_voltage)
        metrics["score"] = score(metrics)
        summaries.append(metrics)

    explicit_summaries = [item for item in summaries if str(item["name"]).startswith("explicit")]
    selected = min(explicit_summaries, key=lambda item: item["score"])
    selected_name = str(selected["name"])
    selected_gain_scale = 0.5 if "half_gains" in selected_name else 1.0
    controller_cfg = config["backstepping_controller"]
    selected_gains = {
        "k1": float(controller_cfg["k1"]) * selected_gain_scale,
        "k2": float(controller_cfg["k2"]) * selected_gain_scale,
        "k3": float(controller_cfg["k3"]) * selected_gain_scale,
        "k4": float(controller_cfg["k4"]) * selected_gain_scale,
    }

    plot_comparison(
        {
            "filtered_baseline": baseline_result,
            selected_name: results[selected_name],
        },
        ARCHIVE_OUTPUT_ROOT / "comparison.png",
    )
    plot_derivative_check(
        results["explicit_hard_half_gains"],
        directional_result,
        ARCHIVE_OUTPUT_ROOT / "derivative_check.png",
    )
    plot_no_filter_history(
        results[selected_name],
        OUTPUT_ROOT / "selected_explicit_state_history.png",
        "Explicit-derivative backstepping",
    )
    animate_pulleys(
        results[selected_name],
        OUTPUT_ROOT / "selected_explicit_animation.gif",
        fps=12,
        title="Explicit-Derivative Backstepping",
        annotation_lines=[
            f"k1={selected_gains['k1']:.1f}, k2={selected_gains['k2']:.1f}",
            f"k3={selected_gains['k3']:.1f}, k4={selected_gains['k4']:.1f}",
        ],
    )

    ranked = sorted(summaries, key=lambda item: item["score"])
    payload = {
        "selected": selected_name,
        "summaries": ranked,
    }
    (OUTPUT_ROOT / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_readme(ranked, selected_name)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
