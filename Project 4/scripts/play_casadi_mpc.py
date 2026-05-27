from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import pygame
except ImportError as exc:
    raise SystemExit(
        "pygame is not installed for this Python interpreter.\n"
        f"Interpreter: {sys.executable}\n"
        f"Install it with: \"{sys.executable}\" -m pip install -r requirements.txt"
    ) from exc

from scripts.play_sample_mpc import (
    COLORS,
    FPS,
    append_telemetry,
    draw_car,
    draw_centrifugal_force,
    draw_driven_path,
    draw_panel,
    draw_plan,
    draw_track,
    window_size,
)
from src.controllers.casadi_mpc import CasadiMPCController
from src.race_env import RaceCarEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the CasADi/IPOPT MPC controller.")
    parser.add_argument("-n", "--horizon", type=int, default=24)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--track-margin", type=float, default=1.00)
    parser.add_argument("--step-delay", type=float, default=0.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = RaceCarEnv(seed=args.seed)
    controller = CasadiMPCController(
        horizon=args.horizon,
        track_margin=args.track_margin,
    )

    pygame.init()
    screen = pygame.display.set_mode(window_size(env))
    pygame.display.set_caption("CasADi Ackermann MPC")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)

    plan = controller.solve(env)
    history: list[dict[str, float]] = []
    append_telemetry(history, env, plan.first_control)

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
                    history.clear()
                    plan = controller.solve(env)
                    append_telemetry(history, env, plan.first_control)

        if not env.done:
            plan = controller.solve(env)
            control = plan.first_control
            env.step(control)
            append_telemetry(history, env, control)
            if args.step_delay > 0.0:
                pygame.time.wait(int(args.step_delay * 1000))

        screen.fill(COLORS["bg"])
        draw_track(screen, env)
        draw_plan(screen, env, plan)
        draw_driven_path(screen, env, history)
        draw_car(screen, env, env.state)
        draw_centrifugal_force(screen, env, env.state)
        draw_panel(screen, env, font, plan, True, history)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
