"""Component registries for planners, controllers, and dynamics backends."""

from __future__ import annotations

from collections.abc import Callable

from .interfaces import Controller, DynamicsBackend, Planner

PlannerFactory = Callable[[], Planner]
ControllerFactory = Callable[[], Controller]
BackendFactory = Callable[[], DynamicsBackend]

PLANNERS: dict[str, PlannerFactory] = {}
CONTROLLERS: dict[str, ControllerFactory] = {}
BACKENDS: dict[str, BackendFactory] = {}

_BUILTINS_REGISTERED = False


def _normalize_planner_name(name: str) -> str:
    key = str(name).lower().strip()
    if key in {"rrt*", "rrtstar", "rrt_star"}:
        return "rrtstar"
    return key


def _normalize_name(name: str) -> str:
    return str(name).lower().strip()


def register_planner(name: str, factory: PlannerFactory) -> None:
    PLANNERS[_normalize_planner_name(name)] = factory


def register_controller(name: str, factory: ControllerFactory) -> None:
    CONTROLLERS[_normalize_name(name)] = factory


def register_backend(name: str, factory: BackendFactory) -> None:
    BACKENDS[_normalize_name(name)] = factory


def create_planner(name: str) -> Planner:
    key = _normalize_planner_name(name)
    if key not in PLANNERS:
        available = ", ".join(sorted(PLANNERS)) or "none"
        raise ValueError(f"Unknown planner '{name}'. Available: {available}")
    return PLANNERS[key]()


def create_controller(name: str) -> Controller:
    key = _normalize_name(name)
    if key not in CONTROLLERS:
        available = ", ".join(sorted(CONTROLLERS)) or "none"
        raise ValueError(f"Unknown controller '{name}'. Available: {available}")
    return CONTROLLERS[key]()


def create_backend(name: str) -> DynamicsBackend:
    key = _normalize_name(name)
    if key not in BACKENDS:
        available = ", ".join(sorted(BACKENDS)) or "none"
        raise ValueError(f"Unknown backend '{name}'. Available: {available}")
    return BACKENDS[key]()


def register_builtin_components() -> None:
    """Register built-in planners/controllers/backends once."""
    global _BUILTINS_REGISTERED
    if _BUILTINS_REGISTERED:
        return

    from ..backends.pointmass_backend import PointMassBackend
    from ..backends.rotorpy_backend import RotorPyBackend
    from ..controllers.basic import LQRController, MPCController, PIDController
    from ..planners.basic import AStarPlanner, RRTPlanner, RRTStarPlanner, StraightPlanner

    register_planner("straight", StraightPlanner)
    register_planner("astar", AStarPlanner)
    register_planner("rrt", RRTPlanner)
    register_planner("rrtstar", RRTStarPlanner)

    register_controller("pid", PIDController)
    register_controller("lqr", LQRController)
    register_controller("mpc", MPCController)

    register_backend("pointmass", PointMassBackend)
    register_backend("rotorpy", RotorPyBackend)

    _BUILTINS_REGISTERED = True
