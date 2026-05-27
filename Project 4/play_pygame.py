from __future__ import annotations

import argparse
from math import cos, sin
import sys
from typing import Iterable, Tuple

try:
    import pygame
except ImportError as exc:
    raise SystemExit(
        "pygame is not installed for this Python interpreter.\n"
        f"Interpreter: {sys.executable}\n"
        f"Install it with: \"{sys.executable}\" -m pip install -r requirements.txt"
    ) from exc

from mpc_controller import DEFAULT_HORIZON, DEFAULT_SAMPLES, MPCPlan, solve_mpc
from race_env import CarState, RaceCarEnv, Control, clamp


BASE_SCALE = 12
MIN_SCALE = 4
MAX_TRACK_VIEW_W = 860
MAX_TRACK_VIEW_H = 700
PADDING = 42
PANEL_W = 360
FPS = 60
MPS_TO_KMH = 3.6
HISTORY_LIMIT = 260

ViewParams = Tuple[float, float, float, float, float, int, int]

COLORS = {
    "bg": (242, 244, 247),
    "grass": (225, 235, 224),
    "track": (66, 73, 82),
    "track_edge": (238, 241, 245),
    "centerline": (247, 194, 66),
    "start": (102, 166, 255),
    "finish": (78, 198, 124),
    "car": (216, 54, 59),
    "car_edge": (126, 26, 34),
    "wheel": (25, 29, 35),
    "front_wheel": (238, 241, 245),
    "target_pose": (78, 198, 124),
    "target_pose_edge": (255, 255, 255),
    "centrifugal": (245, 125, 54),
    "plan": (34, 157, 94),
    "trail": (255, 255, 255),
    "endpoint": (247, 194, 66),
    "panel": (232, 235, 240),
    "plot_bg": (218, 223, 231),
    "plot_grid": (190, 198, 208),
    "speed": (42, 120, 214),
    "gas": (34, 157, 94),
    "steer": (132, 88, 179),
    "torque": (245, 125, 54),
    "gauge_bg": (208, 214, 222),
    "gauge_fill": (34, 157, 94),
    "gauge_warn": (216, 54, 59),
    "text": (35, 42, 52),
    "muted": (105, 115, 130),
    "win": (28, 132, 78),
    "lose": (188, 54, 54),
}


def _view_params(env: RaceCarEnv) -> ViewParams:
    cached = getattr(env, "_pygame_view_params", None)
    if cached is not None:
        return cached

    points = (
        env.track.boundary_points(-1.0, count=260)
        + env.track.boundary_points(1.0, count=260)
    )

    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    margin = 4.0
    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin
    width = max_x - min_x
    height = max_y - min_y
    scale = max(MIN_SCALE, min(BASE_SCALE, MAX_TRACK_VIEW_W / width, MAX_TRACK_VIEW_H / height))
    screen_w = max(720, int(width * scale) + 2 * PADDING + PANEL_W)
    screen_h = max(740, int(height * scale) + 2 * PADDING)
    cached = (min_x, max_x, min_y, max_y, scale, screen_w, screen_h)
    setattr(env, "_pygame_view_params", cached)
    return cached


def world_bounds(env: RaceCarEnv) -> Tuple[float, float, float, float]:
    min_x, max_x, min_y, max_y, _, _, _ = _view_params(env)
    return min_x, max_x, min_y, max_y


def view_scale(env: RaceCarEnv) -> float:
    return _view_params(env)[4]


def window_size(env: RaceCarEnv) -> Tuple[int, int]:
    return _view_params(env)[5], _view_params(env)[6]


def to_screen(env: RaceCarEnv, x: float, y: float) -> Tuple[int, int]:
    min_x, _, _, max_y, scale, _, _ = _view_params(env)
    return (
        int(PADDING + (x - min_x) * scale),
        int(PADDING + (max_y - y) * scale),
    )


def _track_screen_points(
    env: RaceCarEnv,
) -> Tuple[list[Tuple[int, int]], list[Tuple[int, int]], list[Tuple[int, int]]]:
    cached = getattr(env, "_pygame_track_points", None)
    if cached is not None:
        return cached

    left = [to_screen(env, x, y) for x, y in env.track.boundary_points(1.0)]
    right = [to_screen(env, x, y) for x, y in env.track.boundary_points(-1.0)]
    center = [to_screen(env, x, y) for x, y in env.track.centerline_points()]
    cached = (left, right, center)
    setattr(env, "_pygame_track_points", cached)
    return cached


def draw_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    x: int,
    y: int,
    color: Tuple[int, int, int] = COLORS["text"],
) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def draw_track(surface: pygame.Surface, env: RaceCarEnv) -> None:
    min_x, max_x, min_y, max_y, scale, _, _ = _view_params(env)
    world_rect = pygame.Rect(
        PADDING,
        PADDING,
        int((max_x - min_x) * scale),
        int((max_y - min_y) * scale),
    )
    pygame.draw.rect(surface, COLORS["grass"], world_rect)

    left, right, center = _track_screen_points(env)
    pygame.draw.polygon(surface, COLORS["track"], left + list(reversed(right)))
    pygame.draw.lines(surface, COLORS["track_edge"], False, left, width=3)
    pygame.draw.lines(surface, COLORS["track_edge"], False, right, width=3)
    pygame.draw.lines(surface, COLORS["centerline"], False, center, width=1)

    draw_point_marker(
        surface,
        env,
        env.track.centerline_at_s(0.0),
        COLORS["start"],
        radius=7,
    )
    draw_point_marker(
        surface,
        env,
        (env.final_x, env.final_y),
        COLORS["finish"],
        radius=8,
    )
    draw_target_pose(surface, env)


def draw_point_marker(
    surface: pygame.Surface,
    env: RaceCarEnv,
    point: Tuple[float, float],
    color: Tuple[int, int, int],
    radius: int,
) -> None:
    center = to_screen(env, *point)
    pygame.draw.circle(surface, color, center, radius)
    pygame.draw.circle(surface, (255, 255, 255), center, radius, width=2)


def rotated_points(
    center: Tuple[float, float],
    size: Tuple[float, float],
    heading: float,
) -> Iterable[Tuple[int, int]]:
    cx, cy = center
    half_l, half_w = size[0] / 2.0, size[1] / 2.0
    local = [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]
    c, s = cos(heading), sin(heading)
    for x, y in local:
        yield int(cx + x * c - y * s), int(cy + x * s + y * c)


def draw_car(surface: pygame.Surface, env: RaceCarEnv, state: CarState) -> None:
    center = to_screen(env, state.x, state.y)
    scale = view_scale(env)
    car_size = (4.3 * scale, 1.85 * scale)
    # Screen y axis is inverted.
    car_poly = list(rotated_points(center, car_size, -state.psi))
    pygame.draw.polygon(surface, COLORS["car"], car_poly)
    pygame.draw.polygon(surface, COLORS["car_edge"], car_poly, width=2)

    wheelbase_px = env.wheelbase * scale
    track_px = 1.55 * scale
    wheel_size = (0.78 * scale, 0.22 * scale)
    for axle_x, steer in ((-wheelbase_px / 2.0, 0.0), (wheelbase_px / 2.0, state.delta)):
        for side in (-1.0, 1.0):
            local_x = axle_x
            local_y = side * track_px / 2.0
            c, s = cos(-state.psi), sin(-state.psi)
            wx = center[0] + local_x * c - local_y * s
            wy = center[1] + local_x * s + local_y * c
            wheel_poly = list(rotated_points((wx, wy), wheel_size, -(state.psi + steer)))
            pygame.draw.polygon(surface, COLORS["wheel"], wheel_poly)
            if axle_x > 0:
                pygame.draw.polygon(surface, COLORS["front_wheel"], wheel_poly, width=1)


def draw_centrifugal_force(
    surface: pygame.Surface,
    env: RaceCarEnv,
    state: CarState,
) -> None:
    projection = env.track.closest(state.x, state.y)
    curvature = env.track.curvature_at_s(projection.s)
    if abs(curvature) <= 1e-6 or state.v <= 0.2:
        return

    left_x = -sin(projection.psi)
    left_y = cos(projection.psi)
    direction = -1.0 if curvature > 0.0 else 1.0
    ux = direction * left_x
    uy = direction * left_y
    lateral_acc = state.v * state.v * abs(curvature)
    arrow_len = max(18.0, min(58.0, 14.0 + 4.2 * lateral_acc))
    start = to_screen(env, state.x, state.y)
    end = (int(start[0] + ux * arrow_len), int(start[1] - uy * arrow_len))

    pygame.draw.line(surface, COLORS["centrifugal"], start, end, width=5)

    head_len = 13.0
    head_width = 8.0
    vx = ux
    vy = -uy
    px = -vy
    py = vx
    tip = end
    base_x = tip[0] - vx * head_len
    base_y = tip[1] - vy * head_len
    head = [
        tip,
        (int(base_x + px * head_width), int(base_y + py * head_width)),
        (int(base_x - px * head_width), int(base_y - py * head_width)),
    ]
    pygame.draw.polygon(surface, COLORS["centrifugal"], head)


def draw_target_pose(surface: pygame.Surface, env: RaceCarEnv) -> None:
    center = to_screen(env, env.final_x, env.final_y)
    scale = view_scale(env)
    car_size = (4.3 * scale, 1.85 * scale)
    car_poly = list(rotated_points(center, car_size, -env.final_psi))
    target_surface = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    pygame.draw.polygon(target_surface, (*COLORS["target_pose"], 70), car_poly)
    pygame.draw.polygon(target_surface, (*COLORS["target_pose_edge"], 170), car_poly, width=2)
    heading_end = to_screen(
        env,
        env.final_x + 2.2 * cos(env.final_psi),
        env.final_y + 2.2 * sin(env.final_psi),
    )
    pygame.draw.line(target_surface, (*COLORS["target_pose"], 180), center, heading_end, width=4)
    surface.blit(target_surface, (0, 0))


def draw_vertical_gauge(
    surface: pygame.Surface,
    font: pygame.font.Font,
    label: str,
    value: float,
    low: float,
    high: float,
    x: int,
    y: int,
    height: int,
    unit: str,
    warn_threshold: float | None = None,
    limit: float | None = None,
) -> None:
    width = 28
    track = pygame.Rect(x, y, width, height)
    pygame.draw.rect(surface, COLORS["gauge_bg"], track, border_radius=4)
    pygame.draw.rect(surface, (145, 154, 168), track, width=1, border_radius=4)

    normalized = 0.0 if high == low else (value - low) / (high - low)
    normalized = max(0.0, min(1.0, normalized))
    fill_height = int(height * normalized)
    fill = pygame.Rect(x, y + height - fill_height, width, fill_height)
    color = COLORS["gauge_fill"]
    if warn_threshold is not None and abs(value) >= warn_threshold:
        color = COLORS["gauge_warn"]
    pygame.draw.rect(surface, color, fill, border_radius=4)

    zero_y = y + height - int(height * ((0.0 - low) / (high - low)))
    if y <= zero_y <= y + height:
        pygame.draw.line(surface, COLORS["text"], (x - 4, zero_y), (x + width + 4, zero_y), width=1)

    if limit is not None and high != low:
        for marker in (-abs(limit), abs(limit)):
            if low <= marker <= high:
                marker_y = y + height - int(height * ((marker - low) / (high - low)))
                pygame.draw.line(
                    surface,
                    COLORS["gauge_warn"],
                    (x - 8, marker_y),
                    (x + width + 8, marker_y),
                    width=2,
                )
        label_y = y + height - int(height * ((abs(limit) - low) / (high - low)))
        draw_text(surface, font, f"{abs(limit):.0f}", x + width + 10, label_y - 10, COLORS["gauge_warn"])

    draw_text(surface, font, label, x - 10, y - 24, COLORS["text"])
    draw_text(surface, font, f"{value:.0f}{unit}", x - 18, y + height + 8, COLORS["muted"])


def draw_plan(surface: pygame.Surface, env: RaceCarEnv, plan: MPCPlan | None) -> None:
    if plan is None:
        return
    if len(plan.path) >= 2:
        points = [to_screen(env, x, y) for x, y in plan.path]
        pygame.draw.lines(surface, COLORS["plan"], False, points, width=4)
        for point in points[1:]:
            pygame.draw.circle(surface, COLORS["plan"], point, 4)

    step = max(1, len(plan.endpoints) // 100)
    for x, y in plan.endpoints[::step]:
        pygame.draw.circle(surface, COLORS["endpoint"], to_screen(env, x, y), 2)


def append_telemetry(
    history: list[dict[str, float]],
    env: RaceCarEnv,
    control: Control,
) -> None:
    torque_cmd, _ = env.clip_control(control)
    gas = clamp(100.0 * max(0.0, torque_cmd) / env.rear_slip_torque, 0.0, 100.0)
    history.append(
        {
            "time": env.time,
            "x": env.state.x,
            "y": env.state.y,
            "speed": env.state.v * MPS_TO_KMH,
            "gas": gas,
            "steer": env.state.delta,
            "torque": env.state.torque,
        }
    )
    if len(history) > HISTORY_LIMIT:
        del history[: len(history) - HISTORY_LIMIT]


def draw_driven_path(
    surface: pygame.Surface,
    env: RaceCarEnv,
    history: list[dict[str, float]],
) -> None:
    if len(history) < 2:
        return
    points = [to_screen(env, sample["x"], sample["y"]) for sample in history]
    pygame.draw.lines(surface, COLORS["trail"], False, points, width=3)


def draw_timeseries(
    surface: pygame.Surface,
    font: pygame.font.Font,
    title: str,
    value_text: str,
    samples: list[dict[str, float]],
    key: str,
    low: float,
    high: float,
    rect: pygame.Rect,
    color: Tuple[int, int, int],
    limit_lines: Tuple[Tuple[float, str], ...] = (),
) -> None:
    pygame.draw.rect(surface, COLORS["plot_bg"], rect, border_radius=4)
    pygame.draw.rect(surface, (155, 164, 176), rect, width=1, border_radius=4)

    for fraction in (0.25, 0.5, 0.75):
        y = rect.top + int(rect.height * fraction)
        pygame.draw.line(surface, COLORS["plot_grid"], (rect.left, y), (rect.right, y), width=1)

    def y_for(value: float) -> int:
        normalized = 0.0 if high == low else (value - low) / (high - low)
        normalized = clamp(normalized, 0.0, 1.0)
        return rect.bottom - int(normalized * rect.height)

    if low < 0.0 < high:
        zero_y = y_for(0.0)
        pygame.draw.line(surface, COLORS["muted"], (rect.left, zero_y), (rect.right, zero_y), width=1)

    for value, label in limit_lines:
        if low <= value <= high:
            y = y_for(value)
            pygame.draw.line(surface, COLORS["gauge_warn"], (rect.left, y), (rect.right, y), width=2)
            draw_text(surface, font, label, rect.left + 4, y - 18, COLORS["gauge_warn"])

    draw_text(surface, font, title, rect.left, rect.top - 24, COLORS["text"])
    draw_text(surface, font, value_text, rect.right - 112, rect.top - 24, COLORS["muted"])

    if not samples:
        return

    series = samples[-HISTORY_LIMIT:]
    if len(series) == 1:
        x = rect.left
        y = y_for(series[0][key])
        pygame.draw.circle(surface, color, (x, y), 3)
        return

    points = []
    for index, sample in enumerate(series):
        x = rect.left + int(index * rect.width / (len(series) - 1))
        points.append((x, y_for(sample[key])))
    pygame.draw.lines(surface, color, False, points, width=2)
    pygame.draw.circle(surface, color, points[-1], 4)


def draw_panel(
    surface: pygame.Surface,
    env: RaceCarEnv,
    font: pygame.font.Font,
    plan: MPCPlan | None,
    auto_mpc: bool,
    history: list[dict[str, float]],
    mode_label: str = "mpc",
) -> None:
    screen_w, screen_h = window_size(env)
    panel_x = screen_w - PANEL_W + 18
    panel = pygame.Rect(panel_x - 16, PADDING, PANEL_W - 14, screen_h - 2 * PADDING)
    pygame.draw.rect(surface, COLORS["panel"], panel)

    state = env.state
    progress = env.track.progress(state.x, state.y)
    slip_margin = env.rear_slip_torque - abs(state.torque)
    latest = history[-1] if history else {
        "speed": state.v * MPS_TO_KMH,
        "gas": 0.0,
        "steer": state.delta,
        "torque": state.torque,
    }

    draw_text(surface, font, "Ackermann MPC", panel_x, PADDING + 20)
    draw_text(surface, font, f"time: {env.time:.2f}s", panel_x, PADDING + 58, COLORS["muted"])
    draw_text(surface, font, f"mode: {mode_label if auto_mpc else 'manual'}", panel_x, PADDING + 86, COLORS["muted"])
    draw_text(surface, font, f"track: {env.track.name}", panel_x, PADDING + 114, COLORS["muted"])
    draw_text(surface, font, f"progress: {progress:.2f}", panel_x, PADDING + 142, COLORS["muted"])
    draw_text(surface, font, f"slip margin: {slip_margin:.0f} N*m", panel_x, PADDING + 170, COLORS["muted"])

    if env.done:
        status = "WIN" if env.won else "INVALID"
        color = COLORS["win"] if env.won else COLORS["lose"]
    else:
        status = "RUNNING"
        color = COLORS["text"]
    draw_text(surface, font, status, panel_x, PADDING + 204, color)
    if env.reason:
        draw_text(surface, font, env.reason, panel_x + 90, PADDING + 204, COLORS["muted"])

    if plan:
        draw_text(surface, font, f"horizon: {plan.horizon}", panel_x + 162, PADDING + 58, COLORS["muted"])
        draw_text(surface, font, f"samples: {plan.sample_count}", panel_x + 162, PADDING + 86, COLORS["muted"])

    graph_x = panel_x
    graph_w = PANEL_W - 54
    graph_h = 68
    graph_gap = 34
    graph_y = PADDING + 252

    draw_timeseries(
        surface,
        font,
        "speed",
        f"{latest['speed']:.1f} km/h",
        history,
        "speed",
        0.0,
        env.max_speed * MPS_TO_KMH,
        pygame.Rect(graph_x, graph_y, graph_w, graph_h),
        COLORS["speed"],
    )
    graph_y += graph_h + graph_gap
    draw_timeseries(
        surface,
        font,
        "gas",
        f"{latest['gas']:.0f} %",
        history,
        "gas",
        0.0,
        100.0,
        pygame.Rect(graph_x, graph_y, graph_w, graph_h),
        COLORS["gas"],
    )
    graph_y += graph_h + graph_gap
    draw_timeseries(
        surface,
        font,
        "steer",
        f"{latest['steer']:.2f} rad",
        history,
        "steer",
        -env.max_steer,
        env.max_steer,
        pygame.Rect(graph_x, graph_y, graph_w, graph_h),
        COLORS["steer"],
    )
    graph_y += graph_h + graph_gap
    draw_timeseries(
        surface,
        font,
        "wheel torque",
        f"{latest['torque']:.0f} N*m",
        history,
        "torque",
        -env.max_torque_cmd,
        env.max_torque_cmd,
        pygame.Rect(graph_x, graph_y, graph_w, graph_h),
        COLORS["torque"],
        limit_lines=(
            (env.rear_slip_torque, "T slip"),
            (-env.rear_slip_torque, "-T slip"),
        ),
    )


def manual_control(env: RaceCarEnv, key: int) -> Control | None:
    torque = env.state.torque
    steer = env.state.delta
    if key == pygame.K_UP:
        torque = min(0.985 * env.rear_slip_torque, torque + 160.0)
    elif key == pygame.K_DOWN:
        torque = max(-0.35 * env.rear_slip_torque, torque - 180.0)
    elif key == pygame.K_LEFT:
        steer = min(env.max_steer, steer + 0.08)
    elif key == pygame.K_RIGHT:
        steer = max(-env.max_steer, steer - 0.08)
    elif key == pygame.K_SPACE:
        torque = 0.0
    else:
        return None
    return torque, steer


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ackermann racing MPC on a fixed S curve")
    parser.add_argument("-n", "--horizon", type=int, default=DEFAULT_HORIZON)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--manual", action="store_true")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--step-delay", type=float, default=0.08)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    horizon = max(1, min(60, args.horizon))
    samples = max(1, args.samples)
    auto_mpc = not args.manual
    step_delay = max(0.02, args.step_delay)

    pygame.init()
    env = RaceCarEnv(seed=args.seed)
    screen = pygame.display.set_mode(window_size(env))
    pygame.display.set_caption(f"Ackermann Racing MPC - {env.track.label}")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)
    plan = solve_mpc(env, horizon=horizon, samples=samples)
    history: list[dict[str, float]] = []
    append_telemetry(history, env, plan.first_control)
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
                    env.reset()
                    plan = solve_mpc(env, horizon=horizon, samples=samples)
                    history.clear()
                    append_telemetry(history, env, plan.first_control)
                    last_step = pygame.time.get_ticks()
                elif event.key == pygame.K_m:
                    auto_mpc = not auto_mpc
                    last_step = pygame.time.get_ticks()
                elif event.key == pygame.K_LEFTBRACKET:
                    horizon = max(1, horizon - 1)
                    plan = solve_mpc(env, horizon=horizon, samples=samples)
                elif event.key == pygame.K_RIGHTBRACKET:
                    horizon = min(60, horizon + 1)
                    plan = solve_mpc(env, horizon=horizon, samples=samples)
                elif event.key == pygame.K_MINUS:
                    samples = max(20, samples - 20)
                    plan = solve_mpc(env, horizon=horizon, samples=samples)
                elif event.key == pygame.K_EQUALS:
                    samples += 20
                    plan = solve_mpc(env, horizon=horizon, samples=samples)
                else:
                    control = manual_control(env, event.key)
                    if control is not None:
                        env.step(control)
                        plan = solve_mpc(env, horizon=horizon, samples=samples)
                        append_telemetry(history, env, control)
                        last_step = pygame.time.get_ticks()

        now = pygame.time.get_ticks()
        if auto_mpc and not env.done and now - last_step >= int(step_delay * 1000):
            control = plan.first_control
            env.step(control)
            if not env.done:
                plan = solve_mpc(env, horizon=horizon, samples=samples)
            append_telemetry(history, env, control)
            last_step = now

        screen.fill(COLORS["bg"])
        draw_track(screen, env)
        draw_plan(screen, env, plan)
        draw_driven_path(screen, env, history)
        draw_car(screen, env, env.state)
        draw_centrifugal_force(screen, env, env.state)
        draw_panel(screen, env, font, plan, auto_mpc, history)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
