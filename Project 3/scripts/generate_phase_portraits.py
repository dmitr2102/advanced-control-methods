from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
BACKSTEP_ROOT = PROJECT_ROOT / "backstep_explicit_derivatives"
for path in (SRC_ROOT, BACKSTEP_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from config import load_json
from run_explicit_derivatives_backstepping import build_explicit_controller
from run_shaped_step4_case import build_fourth_order_shaper_controller
from simulation import simulate
from tune_pid_controller import build_pid_case


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def selected_summary(summary_path: Path) -> dict:
    payload = read_json(summary_path)
    selected_name = payload["selected"]
    for item in payload["summaries"]:
        if item["name"] == selected_name:
            return item
    raise ValueError(f"Selected case {selected_name!r} is missing in {summary_path}")


def gain_scale_from_name(name: str) -> float:
    if "half_gains" in name:
        return 0.5
    return 1.0


def limit_mode_from_name(name: str) -> str:
    if "no_virtual_limits" in name:
        return "none"
    return "hard"


def reference_velocity(result: dict[str, np.ndarray]) -> np.ndarray:
    details = result["details"]
    if "reference_velocity" in details:
        velocity = np.asarray(details["reference_velocity"], dtype=float)
        if np.all(np.isfinite(velocity)):
            return velocity
    time = np.asarray(result["time"], dtype=float)
    reference = np.asarray(result["reference"], dtype=float)
    return np.gradient(reference, time)


def add_time_colored_trajectory(
    ax,
    x: np.ndarray,
    y: np.ndarray,
    time: np.ndarray,
    linewidth: float = 1.8,
) -> LineCollection:
    points = np.column_stack((x, y)).reshape(-1, 1, 2)
    segments = np.concatenate((points[:-1], points[1:]), axis=1)
    collection = LineCollection(
        segments,
        cmap="viridis",
        norm=plt.Normalize(float(time[0]), float(time[-1])),
    )
    collection.set_array(time[:-1])
    collection.set_linewidth(linewidth)
    ax.add_collection(collection)
    ax.plot(x[0], y[0], "o", color="tab:green", markersize=5, label="start")
    ax.plot(x[-1], y[-1], "o", color="tab:red", markersize=5, label="end")
    ax.autoscale()
    return collection


def add_reference_phase(ax, reference: np.ndarray, ref_velocity: np.ndarray) -> None:
    unique_reference = np.unique(np.round(reference, decimals=10))
    if unique_reference.size <= 6 and np.nanmax(np.abs(ref_velocity)) < 1.0e-9:
        ax.scatter(
            unique_reference,
            np.zeros_like(unique_reference),
            marker="x",
            color="black",
            s=45,
            label="reference setpoints",
        )
        return
    ax.plot(reference, ref_velocity, "k--", linewidth=1.2, label="reference")


def plot_phase_portrait(
    result: dict[str, np.ndarray],
    output_path: Path,
    title: str,
) -> None:
    time = np.asarray(result["time"], dtype=float)
    state = np.asarray(result["state"], dtype=float)
    reference = np.asarray(result["reference"], dtype=float)
    ref_velocity = reference_velocity(result)

    theta_l = state[:, 0]
    omega_l = state[:, 1]
    error = reference - theta_l
    error_dot = ref_velocity - omega_l

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    trajectory = add_time_colored_trajectory(axes[0], theta_l, omega_l, time)
    add_reference_phase(axes[0], reference, ref_velocity)
    axes[0].set_xlabel(r"$\theta_l$ [rad]")
    axes[0].set_ylabel(r"$\omega_l$ [rad/s]")
    axes[0].set_title("load shaft phase")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    add_time_colored_trajectory(axes[1], error, error_dot, time)
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.45)
    axes[1].axvline(0.0, color="black", linewidth=0.8, alpha=0.45)
    axes[1].set_xlabel(r"$e_\theta=\theta_{ref}-\theta_l$ [rad]")
    axes[1].set_ylabel(r"$\dot e_\theta$ [rad/s]")
    axes[1].set_title("tracking error phase")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    fig.colorbar(trajectory, ax=axes, shrink=0.88, label="time [s]")
    fig.suptitle(title, y=0.99)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def generate_pid_phase_portraits(base_config: dict) -> None:
    cases = [
        (
            PROJECT_ROOT / "pid tuning" / "manual_sine" / "manual_sine_summary.json",
            PROJECT_ROOT / "pid tuning" / "manual_sine" / "manual_sine_phase_portrait.png",
            "PID sine phase portrait",
        ),
        (
            PROJECT_ROOT / "pid tuning" / "manual_step" / "manual_step_summary.json",
            PROJECT_ROOT / "pid tuning" / "manual_step" / "manual_step_phase_portrait.png",
            "PID step phase portrait",
        ),
    ]
    for summary_path, output_path, title in cases:
        payload = read_json(summary_path)
        plant, controller, sim_config = build_pid_case(
            base_config,
            payload["mode"],
            float(payload["kp"]),
            float(payload["ki"]),
            float(payload["kd"]),
        )
        result = simulate(plant, controller, sim_config)
        plot_phase_portrait(result, output_path, title)


def generate_backstepping_phase_portraits(base_config: dict) -> None:
    explicit_summary = selected_summary(BACKSTEP_ROOT / "summary.json")
    explicit_name = str(explicit_summary["name"])
    explicit_config = copy.deepcopy(base_config)
    explicit_config["backstepping_controller"]["reference_mode"] = "sine"
    explicit_config["backstepping_controller"]["relative_damping_gain"] = 0.0
    plant, controller, sim_config = build_explicit_controller(
        explicit_config,
        limit_mode_from_name(explicit_name),
        gain_scale_from_name(explicit_name),
    )
    result = simulate(plant, controller, sim_config)
    plot_phase_portrait(
        result,
        BACKSTEP_ROOT / "selected_explicit_phase_portrait.png",
        "Explicit-derivative backstepping phase portrait",
    )

    step_summary = selected_summary(BACKSTEP_ROOT / "step_case" / "step_summary.json")
    step_name = str(step_summary["name"])
    step_config = copy.deepcopy(base_config)
    step_config["backstepping_controller"]["reference_mode"] = "step"
    step_config["backstepping_controller"]["relative_damping_gain"] = 0.0
    plant, controller, sim_config = build_explicit_controller(
        step_config,
        limit_mode_from_name(step_name),
        gain_scale_from_name(step_name),
    )
    result = simulate(plant, controller, sim_config)
    plot_phase_portrait(
        result,
        BACKSTEP_ROOT / "step_case" / "selected_step_phase_portrait.png",
        "Explicit backstepping step phase portrait",
    )

    shaped_summary = selected_summary(
        BACKSTEP_ROOT / "shaped_step4_case" / "shaped_step4_summary.json"
    )
    shaped_name = str(shaped_summary["name"])
    shaped_config = copy.deepcopy(base_config)
    shaped_config["backstepping_controller"]["relative_damping_gain"] = 0.0
    shaped_config["backstepping_controller"]["shaper_frequency"] = float(
        shaped_summary["shaper_frequency_rad_s"]
    )
    plant, controller, sim_config = build_fourth_order_shaper_controller(
        shaped_config,
        limit_mode_from_name(shaped_name),
        float(shaped_summary["gain_scale"]),
    )
    result = simulate(plant, controller, sim_config)
    plot_phase_portrait(
        result,
        BACKSTEP_ROOT / "shaped_step4_case" / "selected_shaped_step4_phase_portrait.png",
        "Fourth-order shaped backstepping phase portrait",
    )


def main() -> None:
    base_config = load_json(PROJECT_ROOT / "configs" / "default.json")
    generate_pid_phase_portraits(base_config)
    generate_backstepping_phase_portraits(base_config)


if __name__ == "__main__":
    main()
