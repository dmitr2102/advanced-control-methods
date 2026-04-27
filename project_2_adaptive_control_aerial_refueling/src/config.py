from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from controllers.adaptive_inverse_mass import AdaptiveInverseMassController
from controllers.pd_controller import NominalMassPDController
from controllers.zero_controller import ZeroController
from simulation import SimulationConfig
from system import AerialRefuelingPlant, RefuelingPlantParameters


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_from_config(config: dict) -> tuple[AerialRefuelingPlant, AdaptiveInverseMassController, SimulationConfig]:
    plant_cfg = config["plant"]
    sim_cfg = config["simulation"]
    controller_cfg = config["controller"]
    initial_cfg = config["initial_state"]

    plant_params = RefuelingPlantParameters(**plant_cfg)
    plant = AerialRefuelingPlant(plant_params)
    controller = AdaptiveInverseMassController(
        damping=plant.damping,
        initial_mass=plant_params.initial_mass,
        gravity=plant_params.gravity,
        lambda_gains=np.array([controller_cfg["lambda_x"], controller_cfg["lambda_z"]], dtype=float),
        k_gains=np.array([controller_cfg["k_x"], controller_cfg["k_z"]], dtype=float),
        gamma=float(controller_cfg["gamma"]),
        mass_hat_initial=float(controller_cfg["mass_hat_initial"]),
        mass_min=float(controller_cfg["mass_min"]),
        mass_max=float(controller_cfg["mass_max"]),
        max_force=float(sim_cfg["max_force"]),
    )
    sim_config = SimulationConfig(
        dt=float(sim_cfg["dt"]),
        horizon=float(sim_cfg["horizon"]),
        refueling_radius=float(sim_cfg["refueling_radius"]),
        initial_state=np.array(
            [initial_cfg["x"], initial_cfg["z"], initial_cfg["vx"], initial_cfg["vz"]],
            dtype=float,
        ),
    )
    return plant, controller, sim_config


def build_baseline_controllers(config: dict, plant: AerialRefuelingPlant) -> dict[str, object]:
    sim_cfg = config["simulation"]
    plant_cfg = config["plant"]
    controller_cfg = config["controller"]
    baseline_cfg = config.get("baseline_controllers", {})
    pd_cfg = baseline_cfg.get("pd", {})
    lambda_gains = np.array([controller_cfg["lambda_x"], controller_cfg["lambda_z"]], dtype=float)

    adaptive_kp = np.array(
        [
            controller_cfg["k_x"] * controller_cfg["lambda_x"],
            controller_cfg["k_z"] * controller_cfg["lambda_z"],
        ],
        dtype=float,
    )
    adaptive_kd = np.array(
        [
            controller_cfg["k_x"] + controller_cfg["lambda_x"],
            controller_cfg["k_z"] + controller_cfg["lambda_z"],
        ],
        dtype=float,
    )

    pd_controller = NominalMassPDController(
        damping=plant.damping,
        nominal_mass=float(plant_cfg["initial_mass"]),
        lambda_gains=lambda_gains,
        kp_gains=np.array(
            [
                pd_cfg.get("kp_x", adaptive_kp[0]),
                pd_cfg.get("kp_z", adaptive_kp[1]),
            ],
            dtype=float,
        ),
        kd_gains=np.array(
            [
                pd_cfg.get("kd_x", adaptive_kd[0]),
                pd_cfg.get("kd_z", adaptive_kd[1]),
            ],
            dtype=float,
        ),
        max_force=float(sim_cfg["max_force"]),
    )
    return {
        "zero": ZeroController(lambda_gains=lambda_gains),
        "pd": pd_controller,
    }
