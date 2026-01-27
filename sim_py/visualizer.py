"""
3D visualization utilities for the standalone simulator.

Uses matplotlib's 3D plotting to show:
- Terrain obstacles
- UAV trajectory
"""

from __future__ import annotations

from typing import Iterable, List, Mapping, Any

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from .terrain_wrapper import BoxObstacle, CylinderObstacle


def plot_simulation(
    trajectory: np.ndarray,
    obstacles: Iterable[CylinderObstacle | BoxObstacle],
    space_dim: np.ndarray | None = None,
    terrain_type: str | None = None,
    visual_cfg: Mapping[str, Any] | None = None,
    planned_waypoints: np.ndarray | None = None,
    goal_position: np.ndarray | None = None,
    planner_type: str | None = None,
    show: bool = True,
) -> plt.Figure:
    """
    Plot UAV trajectory and terrain in 3D.

    Args:
        trajectory: array of shape (N, 3) with ENU positions.
        obstacles: iterable of CylinderObstacle or BoxObstacle.
        space_dim: optional [x, y, z] for axis limits.
        show: whether to call plt.show() at the end.
    """
    traj = np.asarray(trajectory, dtype=float)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Use visual config or defaults
    if visual_cfg is None:
        visual_cfg = {}

    path_linewidth = float(visual_cfg.get("path_linewidth", 2.0))
    marker_size = float(visual_cfg.get("marker_size", 40.0))
    tree_linewidth = float(visual_cfg.get("tree_linewidth", 4.0))
    tree_alpha = float(visual_cfg.get("tree_alpha", 0.9))
    tree_radius_ref = float(visual_cfg.get("tree_radius_ref", 1.0))

    # Plot trajectory
    ax.plot(
        traj[:, 0],
        traj[:, 1],
        traj[:, 2],
        color="blue",
        linewidth=path_linewidth,
        label="UAV path",
        zorder=3,
    )
    ax.scatter(
        traj[0, 0],
        traj[0, 1],
        traj[0, 2],
        color="green",
        s=marker_size,
        label="Start",
        zorder=4,
    )
    # Plot goal position (target)
    if goal_position is not None:
        goal_position = np.asarray(goal_position, dtype=float)
        ax.scatter(
            goal_position[0],
            goal_position[1],
            goal_position[2],
            color="magenta",
            s=marker_size,
            label="Goal",
            zorder=4,
        )

    # Plot planned waypoints if provided (for debugging path planning)
    if planned_waypoints is not None and len(planned_waypoints) > 0:
        planned_waypoints = np.asarray(planned_waypoints, dtype=float)
        if len(planned_waypoints) > 2:
            # Draw planned path as orange dashed line
            ax.plot(
                planned_waypoints[:, 0],
                planned_waypoints[:, 1],
                planned_waypoints[:, 2],
                color="orange",
                linestyle="--",
                linewidth=1.5,
                alpha=0.7,
                label="Planned path",
                zorder=2,
            )
            # Mark waypoints as small orange dots
            ax.scatter(
                planned_waypoints[:, 0],
                planned_waypoints[:, 1],
                planned_waypoints[:, 2],
                color="orange",
                s=20,
                alpha=0.5,
                zorder=2,
            )

    # Plot obstacles
    if terrain_type == "forest":
        # Draw trees as vertical green lines (cylinders approximated)
        for o in obstacles:
            c = o.center
            z0 = float(c[2])
            if hasattr(o, "height"):
                h = float(getattr(o, "height"))
            elif hasattr(o, "size"):
                h = float(getattr(o, "size")[2])
            else:
                h = 2.0
            radius = float(getattr(o, "radius", tree_radius_ref))
            radius_scale = max(0.4, min(3.0, radius / max(tree_radius_ref, 1e-6)))
            ax.plot(
                [float(c[0]), float(c[0])],
                [float(c[1]), float(c[1])],
                [z0, z0 + h],
                color="green",
                linewidth=tree_linewidth * radius_scale,
                alpha=tree_alpha,
            )
    else:
        # Fallback: gray points at obstacle centers
        obs_x: List[float] = []
        obs_y: List[float] = []
        obs_z: List[float] = []
        for o in obstacles:
            c = o.center
            obs_x.append(float(c[0]))
            obs_y.append(float(c[1]))
            z_center = float(c[2])
            if hasattr(o, "height"):
                z_center += float(getattr(o, "height")) / 2.0
            elif hasattr(o, "size"):
                z_center += float(getattr(o, "size")[2]) / 2.0
            obs_z.append(z_center)

        if obs_x:
            ax.scatter(obs_x, obs_y, obs_z, color="gray", alpha=0.6, label="Obstacles")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    title_terrain = (terrain_type or "terrain").capitalize()
    title_planner = (planner_type or "planner").upper()
    ax.set_title(f"{title_terrain} path planner with {title_planner}")

    if space_dim is not None:
        ax.set_xlim(0.0, float(space_dim[0]))
        ax.set_ylim(0.0, float(space_dim[1]))
        ax.set_zlim(0.0, float(space_dim[2]))

    ax.legend()
    ax.view_init(elev=30, azim=135)
    ax.grid(True)

    if show:
        plt.tight_layout()
        plt.show()

    return fig


