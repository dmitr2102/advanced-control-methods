from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Protocol


@dataclass
class ControllerOutput:
    name: str
    action: float
    details: Dict[str, float | str] = field(default_factory=dict)


class Controller(Protocol):
    """Common interface for all controllers used by the simulator."""

    name: str

    def compute_action(self, time_value: float, state: tuple[float, float]) -> ControllerOutput:
        """Return the control action for the current time and state."""
