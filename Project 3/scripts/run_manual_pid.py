from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPTS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from animation_utils import animate_pulleys
from tune_pid_controller import (
    build_pid_case,
    compute_metrics,
    load_json,
    plot_pid_history,
)
from simulation import simulate


# Edit these four values for the fastest manual workflow.
REFERENCE_MODE = "sine"  # "sine" or "step"
KP = 32.2
KI = 0.29
KD = 1.35

RUN_NAME = "manual"
MAKE_ANIMATION = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one manual PID simulation without grid search.",
    )
    parser.add_argument("--mode", choices=("sine", "step"), default=REFERENCE_MODE)
    parser.add_argument("--kp", type=float, default=KP)
    parser.add_argument("--ki", type=float, default=KI)
    parser.add_argument("--kd", type=float, default=KD)
    parser.add_argument("--name", default=RUN_NAME)
    parser.add_argument(
        "--animate",
        action="store_true",
        default=MAKE_ANIMATION,
        help="Also regenerate a GIF animation. Slower than plotting only.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_config = load_json(PROJECT_ROOT / "configs" / "default.json")
    plant, controller, sim_config = build_pid_case(
        base_config,
        args.mode,
        args.kp,
        args.ki,
        args.kd,
    )
    result = simulate(plant, controller, sim_config)
    metrics = compute_metrics(result, args.kp, args.ki, args.kd, args.mode)

    output_dir = PROJECT_ROOT / "pid tuning" / args.name
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_pid_history(
        result,
        output_dir / f"{args.name}_state_history.png",
        f"PID {args.mode} response",
    )
    if args.animate:
        animate_pulleys(
            result,
            output_dir / f"{args.name}_animation.gif",
            fps=12,
            title=f"Manual PID {args.mode}",
            annotation_lines=[
                f"kp={args.kp:.3g}, ki={args.ki:.3g}",
                f"kd={args.kd:.3g}",
            ],
        )

    payload = {
        "mode": args.mode,
        "kp": args.kp,
        "ki": args.ki,
        "kd": args.kd,
        "metrics": metrics,
        "files": {
            "state_history": str(output_dir / f"{args.name}_state_history.png"),
            "summary": str(output_dir / f"{args.name}_summary.json"),
            "animation": str(output_dir / f"{args.name}_animation.gif") if args.animate else None,
        },
    }
    (output_dir / f"{args.name}_summary.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
