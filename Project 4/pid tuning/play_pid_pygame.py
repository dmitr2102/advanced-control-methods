from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPTS_ROOT, Path(__file__).resolve().parent):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import pygame
except ImportError as exc:
    raise SystemExit(
        "pygame is not installed for this Python interpreter.\n"
        f"Interpreter: {sys.executable}\n"
        f"Install it with: \"{sys.executable}\" -m pip install -r requirements.txt"
    ) from exc

from pid_controller import CenterlinePIDController, PIDGains
from play_sample_mpc import (
    COLORS,
    FPS,
    append_telemetry,
    draw_car,
    draw_centrifugal_force,
    draw_driven_path,
    draw_panel,
    draw_track,
    manual_control,
    window_size,
)
from race_env import RaceCarEnv


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ackermann racing PID centerline controller")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--step-delay", type=float, default=0.08)
    parser.add_argument("--target-speed", type=float, default=32.0)
    parser.add_argument("--kp", type=float, default=0.03)
    parser.add_argument("--ki", type=float, default=0.002)
    parser.add_argument("--kd", type=float, default=0.015)
    parser.add_argument("--heading-gain", type=float, default=0.3)
    parser.add_argument("--manual", action="store_true")
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    step_delay = max(0.02, args.step_delay)
    auto_pid = not args.manual

    pygame.init()
    env = RaceCarEnv(seed=args.seed)
    controller = CenterlinePIDController(
        env,
        lateral_gains=PIDGains(kp=args.kp, ki=args.ki, kd=args.kd),
        heading_gain=args.heading_gain,
        target_speed=args.target_speed,
    )
    screen = pygame.display.set_mode(window_size(env))
    pygame.display.set_caption(f"Ackermann Racing PID - {env.track.label}")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)
    history: list[dict[str, float]] = []
    last_control = controller.control()
    append_telemetry(history, env, last_control)
    last_step = pygame.time.get_ticks()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_r:
                    env.reset(args.seed)
                    controller.reset()
                    history.clear()
                    last_control = controller.control()
                    append_telemetry(history, env, last_control)
                    last_step = pygame.time.get_ticks()
                elif event.key == pygame.K_m:
                    auto_pid = not auto_pid
                    last_step = pygame.time.get_ticks()
                else:
                    control = manual_control(env, event.key)
                    if control is not None:
                        env.step(control)
                        last_control = control
                        controller.reset()
                        append_telemetry(history, env, control)
                        last_step = pygame.time.get_ticks()

        now = pygame.time.get_ticks()
        if auto_pid and not env.done and now - last_step >= int(step_delay * 1000):
            last_control = controller.control()
            env.step(last_control)
            append_telemetry(history, env, last_control)
            last_step = now

        screen.fill(COLORS["bg"])
        draw_track(screen, env)
        draw_driven_path(screen, env, history)
        draw_car(screen, env, env.state)
        draw_centrifugal_force(screen, env, env.state)
        draw_panel(screen, env, font, None, auto_pid, history, mode_label="pid")
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
