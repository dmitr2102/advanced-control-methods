from __future__ import annotations

import math
from dataclasses import dataclass

import pygame

from controllers.averaged_energy import AveragedEnergyController
from controllers.harmonic import HarmonicController
from controllers.limit_cycle_lyapunov import LimitCycleLyapunovController
from controllers.lyapunov import LyapunovController
from controllers.pid import CycleEnergyPDController, PositionPDController
from simulation import PendulumSimulator
from system import PendulumParameters


@dataclass
class ViewConfig:
    width: int = 1200
    height: int = 760
    fps: int = 60
    pixels_per_meter: int = 220


class PygameSimulationApp:
    def __init__(
        self,
        simulator: PendulumSimulator,
        harmonic_controller: HarmonicController,
        averaged_energy_controller: AveragedEnergyController,
        lyapunov_controller: LyapunovController,
        limit_cycle_controller: LimitCycleLyapunovController,
        pid_position_controller: PositionPDController,
        pid_cycle_energy_controller: CycleEnergyPDController,
        plant_params: PendulumParameters,
        view: ViewConfig | None = None,
    ) -> None:
        self.simulator = simulator
        self.harmonic_controller = harmonic_controller
        self.averaged_energy_controller = averaged_energy_controller
        self.lyapunov_controller = lyapunov_controller
        self.limit_cycle_controller = limit_cycle_controller
        self.pid_position_controller = pid_position_controller
        self.pid_cycle_energy_controller = pid_cycle_energy_controller
        self.current_controller = harmonic_controller
        self.selected_pid_gain = "kp"
        self.plant_params = plant_params
        self.view = view or ViewConfig()
        self.is_running = True
        self.is_paused = False
        self.message = "1 harmonic, 2 averaged-energy, 3 CLF, 4 orbit-Lyap, 5 PD-pos, 6 PD-energy."

        pygame.init()
        pygame.display.set_caption("Kapitza Pendulum Simulator")
        self.screen = pygame.display.set_mode((self.view.width, self.view.height))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 22)
        self.small_font = pygame.font.SysFont("consolas", 18)

    def run(self) -> None:
        while self.is_running:
            self._handle_events()
            self._update_simulation()
            self._draw()
            self.clock.tick(self.view.fps)
        pygame.quit()

    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.is_running = False
            elif event.type == pygame.KEYDOWN:
                self._handle_keydown(event.key)

    def _handle_keydown(self, key: int) -> None:
        if key == pygame.K_ESCAPE:
            self.is_running = False
        elif key == pygame.K_SPACE:
            self.is_paused = not self.is_paused
            self.message = "Simulation paused." if self.is_paused else "Simulation running."
        elif key == pygame.K_r:
            self.simulator.reset()
            self._reset_all_controllers()
            self.message = "Simulation reset to the initial state."
        elif key == pygame.K_1:
            self.current_controller = self.harmonic_controller
            self._reset_current_controller()
            self.message = "Harmonic controller selected."
        elif key == pygame.K_2:
            self.current_controller = self.averaged_energy_controller
            self._reset_current_controller()
            self.message = "Averaged-energy controller selected."
        elif key == pygame.K_3:
            self.current_controller = self.lyapunov_controller
            self._reset_current_controller()
            self.message = "Closed-loop Lyapunov controller selected."
        elif key == pygame.K_4:
            self.current_controller = self.limit_cycle_controller
            self._reset_current_controller()
            self.message = "Limit-cycle Lyapunov controller selected."
        elif key == pygame.K_5:
            self.current_controller = self.pid_position_controller
            self._reset_current_controller()
            self.message = "PD position controller selected."
        elif key == pygame.K_6:
            self.current_controller = self.pid_cycle_energy_controller
            self._reset_current_controller()
            self.message = "PD cycle-energy controller selected."
        elif key == pygame.K_UP and self.current_controller is self.harmonic_controller:
            self.harmonic_controller.amplitude += 0.5
            self.message = f"Harmonic amplitude increased to {self.harmonic_controller.amplitude:.2f} m/s^2."
        elif key == pygame.K_DOWN and self.current_controller is self.harmonic_controller:
            self.harmonic_controller.amplitude = max(0.0, self.harmonic_controller.amplitude - 0.5)
            self.message = f"Harmonic amplitude decreased to {self.harmonic_controller.amplitude:.2f} m/s^2."
        elif key == pygame.K_RIGHT and self.current_controller is self.harmonic_controller:
            self.harmonic_controller.frequency += 0.5
            self.message = f"Harmonic frequency increased to {self.harmonic_controller.frequency:.2f} rad/s."
        elif key == pygame.K_LEFT and self.current_controller is self.harmonic_controller:
            self.harmonic_controller.frequency = max(0.0, self.harmonic_controller.frequency - 0.5)
            self.message = f"Harmonic frequency decreased to {self.harmonic_controller.frequency:.2f} rad/s."
        elif key == pygame.K_z:
            self.selected_pid_gain = "kp"
            self.message = "PD gain selection: kp."
        elif key == pygame.K_x:
            self.message = "No integral gain in PD control."
        elif key == pygame.K_c:
            self.selected_pid_gain = "kd"
            self.message = "PD gain selection: kd."
        elif key == pygame.K_j:
            self._adjust_pid_gain(-1.0)
        elif key == pygame.K_k:
            self._adjust_pid_gain(1.0)

    def _reset_current_controller(self) -> None:
        reset_fn = getattr(self.current_controller, "reset", None)
        if callable(reset_fn):
            reset_fn()

    def _reset_all_controllers(self) -> None:
        for controller in (
            self.harmonic_controller,
            self.averaged_energy_controller,
            self.lyapunov_controller,
            self.limit_cycle_controller,
            self.pid_position_controller,
            self.pid_cycle_energy_controller,
        ):
            reset_fn = getattr(controller, "reset", None)
            if callable(reset_fn):
                reset_fn()

    def _adjust_pid_gain(self, direction: float) -> None:
        if self.current_controller not in (self.pid_position_controller, self.pid_cycle_energy_controller):
            return
        step_sizes = {"kp": 5.0, "kd": 2.0}
        attr_name = self.selected_pid_gain
        new_value = max(0.0, getattr(self.current_controller, attr_name) + direction * step_sizes[attr_name])
        setattr(self.current_controller, attr_name, new_value)
        self.message = f"{self.current_controller.name}: {attr_name} set to {new_value:.2f}."

    def _update_simulation(self) -> None:
        if self.is_paused:
            return

        substeps = max(1, int((1.0 / self.view.fps) / self.simulator.config.dt))
        for _ in range(substeps):
            self.simulator.step(self.current_controller)

    def _draw(self) -> None:
        self.screen.fill((245, 244, 240))
        self._draw_pendulum()
        self._draw_sidebar()
        pygame.display.flip()

    def _draw_pendulum(self) -> None:
        state = self.simulator.state
        origin_x = int(self.view.width * 0.34)
        origin_y = int(self.view.height * 0.28)
        scale = self.view.pixels_per_meter
        support_offset = int((state.action / max(1.0, self.simulator.config.max_abs_action)) * 80.0)
        support_y = origin_y + support_offset

        rod_length = int(self.plant_params.length * scale)
        phi = state.phi
        bob_x = origin_x - int(rod_length * math.sin(phi))
        bob_y = support_y - int(rod_length * math.cos(phi))

        pygame.draw.line(self.screen, (90, 90, 95), (origin_x - 120, support_y), (origin_x + 120, support_y), 6)
        pygame.draw.circle(self.screen, (70, 70, 70), (origin_x, support_y), 10)
        pygame.draw.line(self.screen, (30, 95, 170), (origin_x, support_y), (bob_x, bob_y), 5)
        pygame.draw.circle(self.screen, (212, 100, 36), (bob_x, bob_y), 24)

        upright_top = (origin_x, support_y - rod_length)
        pygame.draw.circle(self.screen, (90, 170, 90), upright_top, 8)
        pygame.draw.line(self.screen, (150, 190, 150), (origin_x, support_y), upright_top, 2)

    def _draw_sidebar(self) -> None:
        panel_rect = pygame.Rect(int(self.view.width * 0.62), 0, int(self.view.width * 0.38), self.view.height)
        pygame.draw.rect(self.screen, (31, 42, 68), panel_rect)

        lines = [
            "Kapitza Pendulum",
            "",
            f"controller: {self.current_controller.name}",
            f"time: {self.simulator.state.time_value:7.2f} s",
            f"phi: {self.simulator.state.phi:7.3f} rad",
            f"phi_dot: {self.simulator.state.phi_dot:7.3f} rad/s",
            f"action a: {self.simulator.state.action:7.3f} m/s^2",
            "",
            "Controls",
            "1: harmonic controller",
            "2: averaged-energy controller",
            "3: CLF Lyapunov controller",
            "4: orbit Lyapunov controller",
            "5: PD position controller",
            "6: PD cycle-energy controller",
            "UP/DOWN: amplitude +/-",
            "LEFT/RIGHT: frequency +/-",
            "Z/C: choose kp/kd",
            "J/K: selected PD gain -/+",
            "SPACE: pause/resume",
            "R: reset",
            "ESC: quit",
            "",
            "Harmonic parameters",
            f"amplitude: {self.harmonic_controller.amplitude:7.2f} m/s^2",
            f"frequency: {self.harmonic_controller.frequency:7.2f} rad/s",
            "",
            "Averaged-energy parameters",
            f"carrier w: {self.averaged_energy_controller.carrier_frequency:7.2f} rad/s",
            f"k target:  {self.averaged_energy_controller.target_stiffness:7.2f}",
            f"min amp:   {self.averaged_energy_controller.min_amplitude:7.2f} m/s^2",
            f"max amp:   {self.averaged_energy_controller.max_amplitude:7.2f} m/s^2",
            "",
            "Lyapunov parameters",
            f"carrier w: {self.lyapunov_controller.carrier_frequency:7.2f} rad/s",
            f"k0:        {self.lyapunov_controller.stiffness_linear:7.2f}",
            f"k1:        {self.lyapunov_controller.stiffness_cubic:7.2f}",
            f"min amp:   {self.lyapunov_controller.min_amplitude:7.2f} m/s^2",
            f"max amp:   {self.lyapunov_controller.max_amplitude:7.2f} m/s^2",
            "",
            "Orbit Lyapunov parameters",
            f"A*:        {self.limit_cycle_controller.target_amplitude:7.2f} rad",
            f"w*:        {self.limit_cycle_controller.target_frequency:7.2f} rad/s",
            f"carrier w: {self.limit_cycle_controller.carrier_frequency:7.2f} rad/s",
            f"kp:        {self.limit_cycle_controller.position_gain:7.2f}",
            f"kd:        {self.limit_cycle_controller.rate_gain:7.2f}",
            f"kE:        {self.limit_cycle_controller.energy_gain:7.2f}",
            "",
            "PD position parameters",
            f"base amp:  {self.pid_position_controller.base_amplitude:7.2f} m/s^2",
            f"kp:        {self.pid_position_controller.kp:7.2f}",
            f"kd:        {self.pid_position_controller.kd:7.2f}",
            "",
            "PD energy parameters",
            f"base amp:  {self.pid_cycle_energy_controller.base_amplitude:7.2f} m/s^2",
            f"kp:        {self.pid_cycle_energy_controller.kp:7.2f}",
            f"kd:        {self.pid_cycle_energy_controller.kd:7.2f}",
            f"gain edit: {self.selected_pid_gain}",
            "",
            "Model",
            "s = [phi, phi_dot]^T",
            "a = y_ddot",
            "s_dot = p(s, a)",
            "phi_ddot = ((g + a) / l) sin(phi)",
            "",
            "Status",
            self.message,
        ]

        y_pos = 24
        for index, line in enumerate(lines):
            font = self.font if index == 0 else self.small_font
            color = (245, 237, 214) if index == 0 else (230, 235, 245)
            text_surface = font.render(line, True, color)
            self.screen.blit(text_surface, (panel_rect.x + 24, y_pos))
            y_pos += 32 if index == 0 else 24
