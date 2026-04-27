from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ControllerOutput:
    name: str
    force: np.ndarray
    details: dict[str, float] = field(default_factory=dict)
