from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from race_env import RaceCarEnv


@dataclass(frozen=True)
class TrackObstacle:
    s: float
    lateral_center: float
    half_s: float
    half_lateral: float
    label: str

    def contains(self, s: float, lateral: float, margin: float = 0.0) -> bool:
        return (
            abs(s - self.s) <= self.half_s + margin
            and abs(lateral - self.lateral_center) <= self.half_lateral + margin
        )


class ObstacleRaceEnv(RaceCarEnv):
    """S-curve racing environment with apex blockers.

    Each obstacle blocks the inside half of the road around one corner apex,
    forcing the controller to leave the classical racing line and route around
    the object before still finishing fast at the target pose.
    """

    def __init__(self, seed: Optional[int] = None, layout: str = "inside"):
        super().__init__(seed=seed)
        self.layout = layout
        self.obstacles = self._build_obstacles()

    def step(self, control):
        if self.done:
            return self.observation()

        torque_cmd, steer_cmd = self.clip_control(control)
        dt = self.dt / self.substeps

        for _ in range(self.substeps):
            self.integrate_state(self.state, torque_cmd, steer_cmd, dt)

            if self.slipping(self.state):
                self._finish(False, self.slip_reason(self.state))
                break
            if self.collides_with_obstacle(self.state.x, self.state.y):
                self._finish(False, "obstacle collision")
                break
            if self.reached_goal(self.state):
                self._finish(True, "goal")
                break
            if not self.track.inside(self.state.x, self.state.y):
                self._finish(False, "off track")
                break

        self.time += self.dt
        self.step_count += 1
        return self.observation()

    def collides_with_obstacle(self, x: float, y: float, margin: float = 1.15) -> bool:
        projection = self.track.closest(x, y)
        return any(
            obstacle.contains(projection.s, projection.lateral, margin=margin)
            for obstacle in self.obstacles
        )

    def _build_obstacles(self) -> List[TrackObstacle]:
        half_width = self.track.width / 2.0
        result: List[TrackObstacle] = []
        arc_count = 0
        for index, segment in enumerate(self.track.segments):
            if segment.kind != "arc":
                continue
            arc_count += 1
            apex_s = segment.start_s + 0.5 * segment.length
            inside_sign = segment.direction
            obstacle_sign = inside_sign
            if self.layout in {"second_outside", "second_outside_wall"} and arc_count == 2:
                obstacle_sign = -inside_sign
            obstacle_s = apex_s
            half_s = 4.8
            lateral_center = obstacle_sign * half_width * 0.50
            half_lateral = half_width * 0.52
            if self.layout == "second_outside_wall" and arc_count == 2:
                start_s = apex_s - 4.8
                end_s = self.track.length - 28.0
                obstacle_s = 0.5 * (start_s + end_s)
                half_s = 0.5 * (end_s - start_s)
                lateral_center = obstacle_sign * half_width * 0.50
                half_lateral = half_width * 0.50
            result.append(
                TrackObstacle(
                    s=obstacle_s,
                    lateral_center=lateral_center,
                    half_s=half_s,
                    half_lateral=half_lateral,
                    label=f"apex {index} {'outside' if obstacle_sign != inside_sign else 'inside'}",
                )
            )
        return result
