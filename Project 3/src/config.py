from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from controllers.backstepping_controller import BacksteppingController
from simulation import SimulationConfig
from system import CablePulleyPlant, CablePulleyPlantParameters


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_backstepping_from_config(config: dict) -> tuple[CablePulleyPlant, BacksteppingController, SimulationConfig]:
    plant_cfg = config["plant"]
    sim_cfg = config["simulation"]
    reference_cfg = config["controller"]
    controller_cfg = config["backstepping_controller"]
    initial_cfg = config["initial_state"]

    plant = CablePulleyPlant(CablePulleyPlantParameters(**plant_cfg))
    controller = BacksteppingController(
        plant=plant,
        reference_amplitude=float(reference_cfg["reference_amplitude"]),
        reference_frequency=float(reference_cfg["reference_frequency"]),
        reference_mode=str(controller_cfg.get("reference_mode", "sine")),
        step_segment_duration=float(controller_cfg.get("step_segment_duration", 5.0)),
        shaper_frequency=float(controller_cfg.get("shaper_frequency", 2.0)),
        shaper_damping=float(controller_cfg.get("shaper_damping", 1.0)),
        k1=float(controller_cfg["k1"]),
        k2=float(controller_cfg["k2"]),
        k3=float(controller_cfg["k3"]),
        k4=float(controller_cfg["k4"]),
        torque_filter_time=float(controller_cfg["torque_filter_time"]),
        current_filter_time=float(controller_cfg["current_filter_time"]),
        max_torque_command=float(controller_cfg["max_torque_command"]),
        max_current_command=float(controller_cfg["max_current_command"]),
        max_voltage=float(controller_cfg["max_voltage"]),
        relative_damping_gain=float(controller_cfg.get("relative_damping_gain", 0.0)),
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
