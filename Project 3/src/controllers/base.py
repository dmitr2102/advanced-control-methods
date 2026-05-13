from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ControllerOutput:
    name: str
    action: float
    details: dict[str, float] = field(default_factory=dict)
