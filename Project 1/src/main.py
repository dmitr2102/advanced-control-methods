from __future__ import annotations

from controllers.averaged_energy import AveragedEnergyController
from controllers.harmonic import HarmonicController
from controllers.limit_cycle_lyapunov import LimitCycleLyapunovController
from controllers.lyapunov import LyapunovController
from controllers.pid import CycleEnergyPDController, PositionPDController
from simulation import PendulumSimulator, SimulationConfig
from system import KapitzaPendulumPlant, PendulumParameters
from visualization import PygameSimulationApp


def main() -> None:
    params = PendulumParameters()
    plant = KapitzaPendulumPlant(params)
    simulator = PendulumSimulator(
        plant=plant,
        config=SimulationConfig(),
    )

    harmonic_controller = HarmonicController()
    averaged_energy_controller = AveragedEnergyController(
        gravity=params.gravity,
        length=params.length,
    )
    lyapunov_controller = LyapunovController(
        gravity=params.gravity,
        length=params.length,
    )
    limit_cycle_controller = LimitCycleLyapunovController(
        gravity=params.gravity,
        length=params.length,
        damping=params.damping,
    )
    pid_position_controller = PositionPDController()
    pid_cycle_energy_controller = CycleEnergyPDController()

    app = PygameSimulationApp(
        simulator=simulator,
        harmonic_controller=harmonic_controller,
        averaged_energy_controller=averaged_energy_controller,
        lyapunov_controller=lyapunov_controller,
        limit_cycle_controller=limit_cycle_controller,
        pid_position_controller=pid_position_controller,
        pid_cycle_energy_controller=pid_cycle_energy_controller,
        plant_params=params,
    )
    app.run()


if __name__ == "__main__":
    main()
