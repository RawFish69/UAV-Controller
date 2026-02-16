"""Abstract interfaces for planners, controllers, and dynamics backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Iterable, Mapping

import numpy as np

from .types import ControlTarget, SimState, Waypoint


class Planner(ABC):
    """Builds waypoint paths from start to goal."""

    @abstractmethod
    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: Iterable[Any],
        cfg: Mapping[str, Any],
    ) -> list[Waypoint]:
        """Plan a waypoint list from ``start`` to ``goal``."""


class Controller(ABC):
    """Computes control targets for one simulation step."""

    @abstractmethod
    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        """Compute a control target for the current state."""


class DynamicsBackend(ABC):
    """State propagation backend for the simulator."""

    @abstractmethod
    def reset(
        self,
        initial_state: SimState,
        world: Mapping[str, Any],
        cfg: Mapping[str, Any],
    ) -> None:
        """Initialize backend state."""

    @abstractmethod
    def step(self, control_target: ControlTarget, dt: float) -> None:
        """Advance backend state by one time step."""

    @abstractmethod
    def state(self) -> SimState:
        """Get current backend state."""

    def apply_constraints(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
        terrain: Any | None,
        terrain_clearance: float,
    ) -> None:
        """Optional post-step bounds/terrain constraints (default no-op)."""

