"""Core datatypes for the standalone simulator framework."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class SimState:
    """Canonical simulator state used across controllers and backends.

    Quaternion convention is ``[w, x, y, z]`` when present.
    """

    position: np.ndarray
    velocity: np.ndarray
    t: float = 0.0
    attitude_quat: np.ndarray | None = None
    body_rates: np.ndarray | None = None


@dataclass
class ControlTarget:
    """Canonical high-level control target produced by controllers."""

    accel_cmd: np.ndarray
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class Waypoint:
    """Single waypoint in 3D ENU coordinates."""

    position: np.ndarray


@dataclass
class TrajectoryLog:
    """Simulation outputs and diagnostics used for plotting/reporting."""

    trajectory: np.ndarray
    planned_waypoints: np.ndarray
    obstacles: list[Any]
    space_dim: np.ndarray
    terrain_type: str
    visual_cfg: dict[str, Any]
    goal_position: np.ndarray
    planner_type: str
    final_time: float
    final_waypoint_index: int
    final_waypoint_count: int
    distance_to_goal: float
    collisions_detected: int
    attitude_quats: np.ndarray | None = None
    backend_name: str = ""
