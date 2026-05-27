from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
PID_ROOT = Path(__file__).resolve().parent
for path in (SRC_ROOT, SCRIPTS_ROOT, PID_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from PIL import Image

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from pid_controller import CenterlinePIDController, PIDGains
from play_sample_mpc import (
    COLORS,
    append_telemetry,
    draw_car,
    draw_centrifugal_force,
    draw_driven_path,
    draw_panel,
    draw_track,
    window_size,
)
from race_env import RaceCarEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record the full S-curve PID drive as a GIF.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=220)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--scale", type=float, default=0.55)
    parser.add_argument("--frame-stride", type=int, default=2)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--target-speed", type=float, default=32.0)
    parser.add_argument("--kp", type=float, default=0.03)
    parser.add_argument("--ki", type=float, default=0.002)
    parser.add_argument("--kd", type=float, default=0.015)
    parser.add_argument("--heading-gain", type=float, default=0.3)
    return parser.parse_args()


def surface_to_image(surface: pygame.Surface, scale: float) -> Image.Image:
    image = Image.frombytes("RGB", surface.get_size(), pygame.image.tostring(surface, "RGB"))
    if scale != 1.0:
        width = max(1, int(image.width * scale))
        height = max(1, int(image.height * scale))
        image = image.resize((width, height), Image.Resampling.LANCZOS)
    return image


def render_frame(
    surface: pygame.Surface,
    env: RaceCarEnv,
    font: pygame.font.Font,
    history: list[dict[str, float]],
) -> None:
    surface.fill(COLORS["bg"])
    draw_track(surface, env)
    draw_driven_path(surface, env, history)
    draw_car(surface, env, env.state)
    draw_centrifugal_force(surface, env, env.state)
    draw_panel(surface, env, font, None, True, history, mode_label="pid")
    pygame.display.flip()


def main() -> int:
    args = parse_args()
    pygame.init()

    env = RaceCarEnv(seed=args.seed)
    controller = CenterlinePIDController(
        env,
        lateral_gains=PIDGains(kp=args.kp, ki=args.ki, kd=args.kd),
        heading_gain=args.heading_gain,
        target_speed=args.target_speed,
    )
    surface = pygame.display.set_mode(window_size(env))
    font = pygame.font.SysFont("consolas", 18)
    history: list[dict[str, float]] = []
    frames: list[Image.Image] = []

    control = controller.control()
    append_telemetry(history, env, control)
    render_frame(surface, env, font, history)
    frames.append(surface_to_image(surface, args.scale))

    for step in range(args.max_steps):
        if env.done:
            break
        control = controller.control()
        env.step(control)
        append_telemetry(history, env, control)
        if env.done or step % max(1, args.frame_stride) == 0:
            render_frame(surface, env, font, history)
            frames.append(surface_to_image(surface, args.scale))

    output_dir = Path(args.output_dir) if args.output_dir else PID_ROOT
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / (
        f"s_curve_pid_steps{env.step_count}_kp{args.kp:g}_ki{args.ki:g}_kd{args.kd:g}.gif"
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
        f"won={env.won} reason={env.reason}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
