from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
THIS_DIR = PROJECT_ROOT / "backstep_explicit_derivatives"
for path in (SRC_ROOT, SCRIPTS_ROOT, THIS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import numpy as np

from config import build_backstepping_from_config, load_json
from animation_utils import animate_pulleys
from run_explicit_derivatives_backstepping import (
    build_explicit_controller,
    plot_no_filter_history,
    summarize,
)
from simulation import simulate
from tune_pid_controller import step_segment_metrics


OUTPUT_ROOT = PROJECT_ROOT / "backstep_explicit_derivatives" / "step_case"


def step_score(metrics: dict[str, float | str]) -> float:
    voltage_penalty = 0.0015 * float(metrics["rms_voltage_rate_V_s"])
    current_penalty = 0.01 * float(metrics["max_abs_current_A"])
    return float(
        0.35 * float(metrics["rms_angle_error_rad"])
        + 1.25 * float(metrics["step_tail_rms_error_rad"])
        + 0.06 * float(metrics["step_mean_settling_time_s"])
        + 0.10 * float(metrics["rms_highpass_relative_velocity_m_s"])
        + voltage_penalty
        + current_penalty
    )


def enrich_step_metrics(
    name: str,
    result: dict[str, np.ndarray],
    max_voltage: float,
) -> dict[str, float | str]:
    metrics = summarize(name, result, max_voltage)
    metrics.update(step_segment_metrics(result))
    metrics["score"] = step_score(metrics)
    return metrics


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    base_config = load_json(PROJECT_ROOT / "configs" / "default.json")
    config = copy.deepcopy(base_config)
    config["backstepping_controller"]["reference_mode"] = "step"
    config["backstepping_controller"]["relative_damping_gain"] = 0.0
    max_voltage = float(config["backstepping_controller"]["max_voltage"])

    cases = [
        ("explicit_step_hard_half_gains", "hard", 0.5),
        ("explicit_step_hard_same_gains", "hard", 1.0),
        ("explicit_step_no_virtual_limits", "none", 1.0),
    ]

    results: dict[str, dict[str, np.ndarray]] = {}
    summaries: list[dict[str, float | str]] = []

    baseline_plant, baseline_controller, baseline_sim = build_backstepping_from_config(config)
    baseline_result = simulate(baseline_plant, baseline_controller, baseline_sim)
    baseline_metrics = enrich_step_metrics("filtered_step_baseline", baseline_result, max_voltage)
    results["filtered_step_baseline"] = baseline_result
    summaries.append(baseline_metrics)

    for name, limit_mode, gain_scale in cases:
        plant, controller, sim_config = build_explicit_controller(config, limit_mode, gain_scale)
        result = simulate(plant, controller, sim_config)
        results[name] = result
        summaries.append(enrich_step_metrics(name, result, max_voltage))

    explicit_summaries = [
        item for item in summaries if str(item["name"]).startswith("explicit_step")
    ]
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

    plot_no_filter_history(
        results[selected_name],
        OUTPUT_ROOT / "selected_step_state_history.png",
        "Explicit-derivative backstepping step",
        show_tensions=True,
    )
    animate_pulleys(
        results[selected_name],
        OUTPUT_ROOT / "selected_step_animation.gif",
        fps=12,
        title="Explicit Backstepping Step",
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
    (OUTPUT_ROOT / "step_summary.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
