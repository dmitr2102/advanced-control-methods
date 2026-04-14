from __future__ import annotations

import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from horizontal_excitation import (  # noqa: E402
    HorizontalExcitationParameters,
    horizontal_proximity_percent,
    side_equilibria,
)


def main() -> None:
    params = HorizontalExcitationParameters()
    cases = [
        (100.0, 18.0),
        (150.0, 18.0),
        (200.0, 18.0),
        (250.0, 18.0),
        (300.0, 18.0),
    ]

    print("Horizontal excitation analysis")
    print(f"g = {params.gravity:.2f}, l = {params.length:.2f}, d = {params.damping:.2f}")
    print()
    for amplitude, frequency in cases:
        equilibria = side_equilibria(amplitude, frequency, params)
        if equilibria is None:
            print(f"alpha = {amplitude:6.1f}, omega = {frequency:5.1f}: no side equilibria")
            continue
        theta_star = equilibria[0]
        proximity = horizontal_proximity_percent(amplitude, frequency, params)
        print(
            f"alpha = {amplitude:6.1f}, omega = {frequency:5.1f}: "
            f"theta* = {math.degrees(theta_star):6.2f} deg, "
            f"horizontal proximity = {proximity:6.2f}%"
        )


if __name__ == "__main__":
    main()
