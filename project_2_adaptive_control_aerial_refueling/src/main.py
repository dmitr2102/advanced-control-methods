from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from config import build_baseline_controllers, build_from_config, load_json
from simulation import simulate


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    config = load_json(project_root / "configs" / "default.json")
    plant, controller, sim_config = build_from_config(config)
    result = simulate(plant, controller, sim_config)
    baselines = build_baseline_controllers(config, plant)
    baseline_results = {
        "zero": simulate(plant, baselines["zero"], sim_config),
        "pd": simulate(plant, baselines["pd"], sim_config),
    }
    final_state = result["state"][-1]
    summary = {
        "final_state": final_state.tolist(),
        "final_position_norm_m": float(np.linalg.norm(final_state[:2])),
        "final_velocity_norm_m_s": float(np.linalg.norm(final_state[2:])),
        "final_mass_kg": float(result["mass"][-1]),
        "final_mass_hat_kg": float(result["mass_hat"][-1]),
        "final_lyapunov": float(result["lyapunov"][-1]),
        "controller_comparison": {
            name: {
                "final_position_norm_m": float(np.linalg.norm(local_result["state"][-1, :2])),
                "final_velocity_norm_m_s": float(np.linalg.norm(local_result["state"][-1, 2:])),
            }
            for name, local_result in {
                **baseline_results,
                "adaptive": result,
            }.items()
        },
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
