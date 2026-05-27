from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PIL import Image

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from obstacle_mpc.obstacle_controller import ObstacleCasadiMPCController
from obstacle_mpc.obstacle_env import ObstacleRaceEnv
from obstacle_mpc.play_obstacles import draw_obstacles
from play_pygame import (
    COLORS,
    append_telemetry,
    draw_car,
    draw_centrifugal_force,
    draw_driven_path,
    draw_panel,
    draw_plan,
    draw_track,
    window_size,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record obstacle CasADi MPC as GIF.")
    parser.add_argument("-n", "--horizon", type=int, default=15)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--layout",
        choices=("inside", "second_outside", "second_outside_wall"),
        default="inside",
    )
    parser.add_argument("--track-margin", type=float, default=1.20)
    parser.add_argument("--max-steps", type=int, default=170)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--scale", type=float, default=0.55)
    parser.add_argument("--frame-stride", type=int, default=2)
    parser.add_argument("--output-dir", default="outputs")
    return parser.parse_args()


def surface_to_image(surface: pygame.Surface, scale: float) -> Image.Image:
    image = Image.frombytes("RGB", surface.get_size(), pygame.image.tostring(surface, "RGB"))
    if scale != 1.0:
        width = max(1, int(image.width * scale))
        height = max(1, int(image.height * scale))
        image = image.resize((width, height), Image.Resampling.LANCZOS)
    return image


def render_frame(surface, env, font, plan, history) -> None:
    surface.fill(COLORS["bg"])
    draw_track(surface, env)
    draw_obstacles(surface, env)
    draw_plan(surface, env, plan)
    draw_driven_path(surface, env, history)
    draw_car(surface, env, env.state)
    draw_centrifugal_force(surface, env, env.state)
    draw_panel(surface, env, font, plan, True, history)
    pygame.display.flip()


def main() -> int:
    args = parse_args()
    pygame.init()

    env = ObstacleRaceEnv(seed=args.seed, layout=args.layout)
    controller = ObstacleCasadiMPCController(
        horizon=args.horizon,
        track_margin=args.track_margin,
    )
    surface = pygame.display.set_mode(window_size(env))
    font = pygame.font.SysFont("consolas", 18)
    history: list[dict[str, float]] = []
    frames: list[Image.Image] = []

    plan = controller.solve(env)
    append_telemetry(history, env, plan.first_control)
    render_frame(surface, env, font, plan, history)
    frames.append(surface_to_image(surface, args.scale))

    for step in range(args.max_steps):
        if env.done:
            break
        control = plan.first_control
        env.step(control)
        append_telemetry(history, env, control)
        plan = controller.solve(env)
        if env.done or step % max(1, args.frame_stride) == 0:
            render_frame(surface, env, font, plan, history)
            frames.append(surface_to_image(surface, args.scale))

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / (
        f"s_curve_obstacles_{args.layout}_casadi_h{args.horizon}_steps{env.step_count}.gif"
    )
    duration_ms = max(1, int(1000 / max(1, args.fps)))
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=False,
    )

    pygame.quit()
    print(
        f"saved {output} frames={len(frames)} steps={env.step_count} "
        f"horizon={args.horizon} won={env.won} reason={env.reason}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
