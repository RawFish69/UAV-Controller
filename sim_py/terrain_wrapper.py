"""
Terrain generation wrapper for the standalone Python simulator.

This reuses the existing ROS2 terrain_generator Python package without
requiring rclpy. We import the generator functions and obstacle classes
directly and drive them using a YAML config.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Literal, Sequence

import numpy as np
import yaml


# Ensure we can import the existing terrain_generator package.
REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_SRC_CANDIDATES = [
    REPO_ROOT / "ros2_ws" / "src",
]
for _ros2_src in ROS2_SRC_CANDIDATES:
    if _ros2_src.exists() and str(_ros2_src) not in sys.path:
        sys.path.append(str(_ros2_src))

from terrain_generator.terrain_generator.forest_generator import generate_forest  # type: ignore  # noqa: E402
from terrain_generator.terrain_generator.mountains_generator import (  # type: ignore  # noqa: E402,E501
    generate_mountains,
)
from terrain_generator.terrain_generator.plains_generator import (  # type: ignore  # noqa: E402,E501
    generate_plains,
)
from terrain_generator.terrain_generator.obstacles import (  # type: ignore  # noqa: E402,E501
    BoxObstacle,
    CylinderObstacle,
    HeightFieldTerrain,
    is_point_in_collision,
)


TerrainType = Literal["forest", "mountains", "plains"]


@dataclass
class TerrainConfig:
    terrain_type: TerrainType = "forest"
    space_dim: np.ndarray = field(default_factory=lambda: np.array([100.0, 100.0, 50.0]))
    start_pos: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    # Nested dicts for terrain-specific params; kept generic to closely
    # mirror the ROS2 YAML structure.
    forest: dict | None = None
    mountains: dict | None = None
    plains: dict | None = None


def load_terrain_config(
    yaml_path: Path | str | None = None,
) -> TerrainConfig:
    """
    Load terrain configuration from YAML.

    If no path is given, defaults to the ROS2 terrain_params.yaml.
    """
    if yaml_path is None:
        candidates = [
            REPO_ROOT / "ros2_ws" / "src" / "terrain_generator" / "config" / "terrain_params.yaml",
        ]
        yaml_path = next((c for c in candidates if c.exists()), candidates[0])
    yaml_path = Path(yaml_path)

    # Explicit UTF-8 to avoid Windows cp1252 decode issues
    with yaml_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    # Expect the same top-level key as the ROS2 node.
    params = raw.get("terrain_generator", {}).get("ros__parameters", {})

    terrain_type: TerrainType = params.get("terrain_type", "forest")
    space_dim = np.array(params.get("space_dim", [100.0, 100.0, 50.0]), dtype=float)
    start_pos = np.array(params.get("start_pos", [0.0, 0.0, 0.0]), dtype=float)

    return TerrainConfig(
        terrain_type=terrain_type,
        space_dim=space_dim,
        start_pos=start_pos,
        forest=params.get("forest"),
        mountains=params.get("mountains"),
        plains=params.get("plains"),
    )


def generate_terrain(
    cfg: TerrainConfig,
    forest_density_scale: float = 1.0,
    tree_height_scale: float = 1.0,
) -> List[CylinderObstacle | BoxObstacle | HeightFieldTerrain]:
    """Generate a list of obstacles using the existing generators."""
    obstacles: List[CylinderObstacle | BoxObstacle | HeightFieldTerrain] = []

    if cfg.terrain_type == "forest":
        p = cfg.forest or {}
        grid_size = int(p.get("grid_size", 10))
        radius_range = tuple(p.get("radius_range", [0.5, 1.5]))
        base_height_range = tuple(p.get("height_range", [5.0, 15.0]))
        height_range = tuple(float(h) * float(tree_height_scale) for h in base_height_range)
        base_density = float(p.get("density", 0.7))
        density = max(0.0, min(1.0, base_density * float(forest_density_scale)))

        obstacles = generate_forest(
            cfg.space_dim,
            grid_size=grid_size,
            radius_range=radius_range,
            height_range=height_range,
            density=density,
            start_pos=cfg.start_pos,
        )

    elif cfg.terrain_type == "mountains":
        p = cfg.mountains or {}
        num_peaks = int(p.get("num_peaks", 15))
        base_size_range = tuple(p.get("base_size_range", [3.0, 8.0]))
        height_range = tuple(p.get("height_range", [10.0, 30.0]))
        grid_resolution = float(p.get("grid_resolution", 10.0))
        ridge_count = int(p.get("ridge_count", 2))
        ridge_chain_count = int(p.get("ridge_chain_count", 2))
        ridge_chain_points = int(p.get("ridge_chain_points", 4))
        ridge_chain_width = float(p.get("ridge_chain_width", 18.0))
        ridge_chain_peak_spacing = float(p.get("ridge_chain_peak_spacing", 18.0))
        ridge_chain_peak_boost = float(p.get("ridge_chain_peak_boost", 1.0))
        pit_count = int(p.get("pit_count", 6))
        pit_depth_range = tuple(p.get("pit_depth_range", [5.0, 20.0]))
        steepness = float(p.get("steepness", 1.6))
        min_height = float(p.get("min_height", 0.5))

        obstacles = generate_mountains(
            cfg.space_dim,
            num_peaks=num_peaks,
            base_size_range=base_size_range,
            height_range=height_range,
            start_pos=cfg.start_pos,
            grid_resolution=grid_resolution,
            ridge_count=ridge_count,
            ridge_chain_count=ridge_chain_count,
            ridge_chain_points=ridge_chain_points,
            ridge_chain_width=ridge_chain_width,
            ridge_chain_peak_spacing=ridge_chain_peak_spacing,
            ridge_chain_peak_boost=ridge_chain_peak_boost,
            pit_count=pit_count,
            pit_depth_range=pit_depth_range,
            steepness=steepness,
            min_height=min_height,
        )

    elif cfg.terrain_type == "plains":
        p = cfg.plains or {}
        num_obstacles = int(p.get("num_obstacles", 10))
        obstacle_types: Sequence[str] = p.get(
            "obstacle_types", ["bush", "rock"]
        )

        obstacles = generate_plains(
            cfg.space_dim,
            num_obstacles=num_obstacles,
            obstacle_types=list(obstacle_types),
            start_pos=cfg.start_pos,
        )

    else:
        raise ValueError(f"Unknown terrain_type: {cfg.terrain_type}")

    return obstacles


__all__ = [
    "TerrainConfig",
    "generate_terrain",
    "load_terrain_config",
    "CylinderObstacle",
    "BoxObstacle",
    "HeightFieldTerrain",
    "is_point_in_collision",
]

