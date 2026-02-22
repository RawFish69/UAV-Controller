from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np

from .repo_paths import ensure_repo_root_on_path


def _resolve_terrain_config(repo_root: Path) -> Path:
    candidates = [
        repo_root / 'ros2_ws' / 'src' / 'terrain_generator' / 'config' / 'terrain_params.yaml',
        repo_root / 'ros2_ws_legacy' / 'src' / 'terrain_generator' / 'config' / 'terrain_params.yaml',
    ]
    for c in candidates:
        if c.exists():
            return c
    # Fall back to the original expected path; the caller will get a useful error.
    return candidates[0]


def plan_trajectory_points(
    start_xyz: Iterable[float],
    goal_xyz: Iterable[float],
    planner_type: str = 'rrtstar',
    terrain_profile: str = 'plains',
    collision_inflation_m: float = 0.5,
) -> list[np.ndarray]:
    """Return a list of ENU waypoint positions using the existing planner stack."""
    repo_root = ensure_repo_root_on_path()

    from sim_py.planner import default_plan  # type: ignore
    from sim_py.terrain_wrapper import generate_terrain, load_terrain_config  # type: ignore

    terrain_cfg = load_terrain_config(_resolve_terrain_config(repo_root))
    if terrain_profile:
        terrain_cfg.terrain_type = str(terrain_profile).lower()

    obstacles = generate_terrain(terrain_cfg)
    planner_key = str(planner_type).lower().strip()
    if planner_key in {'rrt*', 'rrt_star'}:
        planner_key = 'rrtstar'

    waypoints = default_plan(
        space_dim=np.asarray(terrain_cfg.space_dim, dtype=float),
        start_pos=np.asarray(terrain_cfg.start_pos, dtype=float),
        obstacles=obstacles,
        path_cfg={
            'planner_type': planner_key,
            'start_abs': list(np.asarray(start_xyz, dtype=float).reshape(3)),
            'goal_abs': list(np.asarray(goal_xyz, dtype=float).reshape(3)),
            'collision_inflation': float(collision_inflation_m),
        },
    )
    return [np.asarray(wp.position, dtype=float).reshape(3) for wp in waypoints]
