from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT / "src"
SCRIPTS_ROOT = ROOT / "scripts"
for path in (SRC_ROOT, SCRIPTS_ROOT):
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

from controllers.obstacle_mpc import ObstacleCasadiMPCController
from obstacle_env import ObstacleRaceEnv, TrackObstacle
from play_sample_mpc import (
    COLORS,
    FPS,
    append_telemetry,
    draw_car,
    draw_centrifugal_force,
    draw_driven_path,
    draw_panel,
    draw_plan,
    draw_track,
    to_screen,
    window_size,
)


OBSTACLE_COLOR = (200, 45, 54)
OBSTACLE_EDGE = (255, 235, 235)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CasADi MPC with apex obstacles.")
    parser.add_argument("-n", "--horizon", type=int, default=15)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--layout",
        choices=("inside", "second_outside", "second_outside_wall"),
        default="inside",
    )
    parser.add_argument("--track-margin", type=float, default=1.20)
    parser.add_argument("--step-delay", type=float, default=0.0)
    return parser.parse_args()


def obstacle_polygon(env: ObstacleRaceEnv, obstacle: TrackObstacle):
    start_s = max(0.0, obstacle.s - obstacle.half_s)
    end_s = min(env.track.length, obstacle.s + obstacle.half_s)
    inner_lateral = obstacle.lateral_center - obstacle.half_lateral
    outer_lateral = obstacle.lateral_center + obstacle.half_lateral
    sample_count = max(8, int((end_s - start_s) / 1.4))

    outer_edge = []
    inner_edge = []
    for index in range(sample_count + 1):
        alpha = index / sample_count
        s = start_s + alpha * (end_s - start_s)
        outer_edge.append(to_screen(env, *env.track.offset_point(s, outer_lateral)))
        inner_edge.append(to_screen(env, *env.track.offset_point(s, inner_lateral)))
    return outer_edge + list(reversed(inner_edge))


def draw_obstacles(surface: pygame.Surface, env: ObstacleRaceEnv) -> None:
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for obstacle in env.obstacles:
        polygon = obstacle_polygon(env, obstacle)
        pygame.draw.polygon(overlay, (*OBSTACLE_COLOR, 215), polygon)
        pygame.draw.polygon(overlay, (*OBSTACLE_EDGE, 245), polygon, width=2)
    surface.blit(overlay, (0, 0))


def main() -> None:
    args = parse_args()
    env = ObstacleRaceEnv(seed=args.seed, layout=args.layout)
    controller = ObstacleCasadiMPCController(
        horizon=args.horizon,
        track_margin=args.track_margin,
    )

    pygame.init()
    screen = pygame.display.set_mode(window_size(env))
    pygame.display.set_caption("Obstacle CasADi Racing MPC")
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
                    env = ObstacleRaceEnv(seed=args.seed, layout=args.layout)
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
        draw_obstacles(screen, env)
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
