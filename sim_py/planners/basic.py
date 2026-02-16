"""Built-in planner implementations for the simulation framework."""

from __future__ import annotations

from typing import Any, Iterable, Mapping

import numpy as np

from ..core.interfaces import Planner
from ..core.types import Waypoint
from ..planner import default_plan as legacy_default_plan


class _LegacyPlannerAdapter(Planner):
    """Adapter around existing planner implementation in ``sim_py.planner``."""

    def __init__(self, planner_type: str):
        self._planner_type = planner_type

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: Iterable[Any],
        cfg: Mapping[str, Any],
    ) -> list[Waypoint]:
        path_cfg = dict(cfg.get("path", {}) or {})
        space_dim = np.asarray(cfg.get("space_dim", np.array([100.0, 100.0, 50.0])), dtype=float)
        start_pos = np.asarray(cfg.get("start_pos", np.array([0.0, 0.0, 0.0])), dtype=float)

        # Keep legacy planner behavior while forcing selected planner type.
        path_cfg["planner_type"] = "rrt*" if self._planner_type == "rrtstar" else self._planner_type
        path_cfg["start_abs"] = np.asarray(start, dtype=float).tolist()
        path_cfg["goal_abs"] = np.asarray(goal, dtype=float).tolist()

        legacy_waypoints = legacy_default_plan(
            space_dim=space_dim,
            start_pos=start_pos,
            obstacles=list(obstacles),
            path_cfg=path_cfg,
        )

        return [
            Waypoint(position=np.asarray(wp.position, dtype=float))
            for wp in legacy_waypoints
        ]


class StraightPlanner(_LegacyPlannerAdapter):
    def __init__(self) -> None:
        super().__init__("straight")


class AStarPlanner(_LegacyPlannerAdapter):
    def __init__(self) -> None:
        super().__init__("astar")


class RRTPlanner(_LegacyPlannerAdapter):
    def __init__(self) -> None:
        super().__init__("rrt")


class RRTStarPlanner(_LegacyPlannerAdapter):
    def __init__(self) -> None:
        super().__init__("rrtstar")
