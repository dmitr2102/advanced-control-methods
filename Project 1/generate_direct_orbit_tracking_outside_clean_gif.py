from __future__ import annotations

import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.direct_orbit_tracking import DirectOrbitTrackingController  # noqa: E402
from simulation import PendulumSimulator, SimulationConfig  # noqa: E402
from system import KapitzaPendulumPlant, PendulumParameters  # noqa: E402


def draw_clean_frame(
    width: int,
    height: int,
    pivot: tuple[int, int],
    rod_length_px: int,
    phi: float,
    phi_ref: float,
    ref_path: list[tuple[int, int]],
) -> Image.Image:
    image = Image.new("RGBA", (width, height), (245, 244, 240, 255))

    pivot_x, pivot_y = pivot
    actual_bob = (
        pivot_x - int(rod_length_px * math.sin(phi)),
        pivot_y - int(rod_length_px * math.cos(phi)),
    )
    ref_bob = (
        pivot_x - int(rod_length_px * math.sin(phi_ref)),
        pivot_y - int(rod_length_px * math.cos(phi_ref)),
    )
    ref_path.append(ref_bob)

    draw = ImageDraw.Draw(image)

    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    if len(ref_path) > 1:
        overlay_draw.line(ref_path, fill=(140, 168, 112, 100), width=6)
    overlay_draw.line((pivot_x, pivot_y, ref_bob[0], ref_bob[1]), fill=(140, 168, 112, 110), width=7)
    ref_bob_radius = 31
    overlay_draw.ellipse(
        (ref_bob[0] - ref_bob_radius, ref_bob[1] - ref_bob_radius, ref_bob[0] + ref_bob_radius, ref_bob[1] + ref_bob_radius),
        fill=(140, 168, 112, 110),
    )
    image.alpha_composite(overlay)

    draw.line((pivot_x - 120, pivot_y, pivot_x + 120, pivot_y), fill=(96, 97, 102), width=8)
    draw.line((pivot_x, pivot_y, pivot_x, pivot_y - rod_length_px), fill=(182, 212, 182), width=2)
    draw.ellipse((pivot_x - 8, pivot_y - 8, pivot_x + 8, pivot_y + 8), fill=(70, 70, 70))
    draw.line((pivot_x, pivot_y, actual_bob[0], actual_bob[1]), fill=(38, 98, 170), width=8)
    actual_bob_radius = 22
    draw.ellipse(
        (
            actual_bob[0] - actual_bob_radius,
            actual_bob[1] - actual_bob_radius,
            actual_bob[0] + actual_bob_radius,
            actual_bob[1] + actual_bob_radius,
        ),
        fill=(212, 100, 36),
        outline=(120, 50, 20),
    )

    return image.convert("P", palette=Image.ADAPTIVE)


def main() -> None:
    output_path = PROJECT_ROOT / "animations" / "direct_orbit_tracking_outside_clean.gif"
    output_path.parent.mkdir(exist_ok=True)

    params = PendulumParameters(damping=0.18)
    controller = DirectOrbitTrackingController(
        gravity=params.gravity,
        length=params.length,
        damping=params.damping,
        target_amplitude=0.14,
        target_frequency=1.45,
        position_gain=12.0,
        rate_gain=4.0,
        orbit_gain=10.0,
        phase_gain=0.0,
        regularization_eps=0.08,
        max_abs_action=140.0,
    )
    simulator = PendulumSimulator(
        KapitzaPendulumPlant(params),
        SimulationConfig(
            dt=1.0 / 240.0,
            initial_phi=0.22,
            initial_phi_dot=0.0,
            max_abs_action=controller.max_abs_action,
        ),
    )

    width, height = 900, 900
    pivot = (width // 2, height // 2 + 260)
    rod_length_px = int(560 * params.length)
    frame_step = 12
    horizon = 15.0
    ref_path: list[tuple[int, int]] = []
    frames: list[Image.Image] = []

    for step in range(int(horizon / simulator.config.dt)):
        state = simulator.step(controller)
        phi_ref, _, _ = controller.reference_state(state.time_value)
        if step % frame_step == 0:
            frames.append(
                draw_clean_frame(
                    width=width,
                    height=height,
                    pivot=pivot,
                    rod_length_px=rod_length_px,
                    phi=state.phi,
                    phi_ref=phi_ref,
                    ref_path=ref_path,
                )
            )

    if not frames:
        raise RuntimeError("No frames generated for direct_orbit_tracking_outside_clean.gif")

    frame_duration_ms = int(1000.0 * simulator.config.dt * frame_step)
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=frame_duration_ms,
        loop=0,
        disposal=2,
    )
    print(f"Saved GIF to: {output_path}")


if __name__ == "__main__":
    main()
