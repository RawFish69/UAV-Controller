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

from .terrain_wrapper import BoxObstacle, CylinderObstacle, HeightFieldTerrain


def _quat_wxyz_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] into a 3x3 rotation matrix."""
    q = np.asarray(quat_wxyz, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def plot_simulation(
    trajectory: np.ndarray,
    obstacles: Iterable[CylinderObstacle | BoxObstacle],
    space_dim: np.ndarray | None = None,
    terrain_type: str | None = None,
    visual_cfg: Mapping[str, Any] | None = None,
    planned_waypoints: np.ndarray | None = None,
    goal_position: np.ndarray | None = None,
    planner_type: str | None = None,
    attitude_quats: np.ndarray | None = None,
    backend_name: str | None = None,
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

    # Higher contrast defaults to keep path visible through terrain
    path_linewidth = float(visual_cfg.get("path_linewidth", 2.0))
    path_color = str(visual_cfg.get("path_color", "deepskyblue"))
    marker_size = float(visual_cfg.get("marker_size", 40.0))
    tree_linewidth = float(visual_cfg.get("tree_linewidth", 4.0))
    tree_alpha = float(visual_cfg.get("tree_alpha", 0.9))
    tree_radius_ref = float(visual_cfg.get("tree_radius_ref", 1.0))
    planned_linewidth = float(visual_cfg.get("planned_linewidth", 2.0))
    planned_alpha = float(visual_cfg.get("planned_alpha", 0.8))
    terrain_alpha = float(visual_cfg.get("terrain_alpha", 0.45))
    terrain_cmap = str(visual_cfg.get("terrain_cmap", "terrain"))
    base_span = 100.0
    if space_dim is not None:
        space_dim_arr = np.asarray(space_dim, dtype=float).reshape(3)
        base_span = float(max(space_dim_arr[0], space_dim_arr[1], 1.0))
    auto_quad_len = max(1.2, 0.02 * base_span)
    quad_arm_length = float(visual_cfg.get("quad_arm_length", auto_quad_len))
    attitude_axis_scale = float(visual_cfg.get("attitude_axis_scale", quad_arm_length * 0.6))
    quad_max_frames = int(visual_cfg.get("quad_max_frames", 35))

    # Plot trajectory
    ax.plot(
        traj[:, 0],
        traj[:, 1],
        traj[:, 2],
        color=path_color,
        linewidth=path_linewidth,
        label="UAV path",
        zorder=5,
    )
    ax.scatter(
        traj[0, 0],
        traj[0, 1],
        traj[0, 2],
        color="cyan",
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
                linewidth=planned_linewidth,
                alpha=planned_alpha,
                label="Planned path",
                zorder=4,
            )
            # Mark waypoints as small orange dots
            ax.scatter(
                planned_waypoints[:, 0],
                planned_waypoints[:, 1],
                planned_waypoints[:, 2],
                color="orange",
                s=20,
                alpha=0.5,
                zorder=4,
            )

    # Optional rotorcraft pose overlay from quaternion trajectory.
    if attitude_quats is not None:
        q_traj = np.asarray(attitude_quats, dtype=float)
        if q_traj.ndim == 2 and q_traj.shape[1] == 4:
            n = min(len(traj), len(q_traj))
            if n > 0:
                stride = max(1, n // max(1, quad_max_frames))
                first_label = True
                for i in range(0, n, stride):
                    q = q_traj[i]
                    if not np.all(np.isfinite(q)):
                        continue
                    p = traj[i]
                    R = _quat_wxyz_to_rotmat(q)

                    # Body axes in world frame.
                    ex = R[:, 0]
                    ey = R[:, 1]
                    ez = R[:, 2]

                    # Draw a simple quad "X"/"+" body from x/y axes.
                    arm_x0 = p - ex * quad_arm_length * 0.5
                    arm_x1 = p + ex * quad_arm_length * 0.5
                    arm_y0 = p - ey * quad_arm_length * 0.5
                    arm_y1 = p + ey * quad_arm_length * 0.5

                    ax.plot(
                        [arm_x0[0], arm_x1[0]],
                        [arm_x0[1], arm_x1[1]],
                        [arm_x0[2], arm_x1[2]],
                        color="black",
                        linewidth=2.4,
                        alpha=0.95,
                        label="Quad pose" if first_label else None,
                        zorder=6,
                    )
                    ax.plot(
                        [arm_y0[0], arm_y1[0]],
                        [arm_y0[1], arm_y1[1]],
                        [arm_y0[2], arm_y1[2]],
                        color="black",
                        linewidth=2.4,
                        alpha=0.95,
                        zorder=6,
                    )

                    # Draw body-frame axes (X red, Y green, Z blue).
                    ax.quiver(
                        p[0], p[1], p[2],
                        ex[0], ex[1], ex[2],
                        length=attitude_axis_scale,
                        color="red",
                        alpha=0.7,
                        linewidth=1.0,
                    )
                    ax.quiver(
                        p[0], p[1], p[2],
                        ey[0], ey[1], ey[2],
                        length=attitude_axis_scale,
                        color="lime",
                        alpha=0.7,
                        linewidth=1.0,
                    )
                    ax.quiver(
                        p[0], p[1], p[2],
                        ez[0], ez[1], ez[2],
                        length=attitude_axis_scale,
                        color="dodgerblue",
                        alpha=0.7,
                        linewidth=1.0,
                    )
                    first_label = False

                # Always highlight the final quad pose to make it easy to spot.
                qf = q_traj[n - 1]
                if np.all(np.isfinite(qf)):
                    pf = traj[n - 1]
                    Rf = _quat_wxyz_to_rotmat(qf)
                    exf = Rf[:, 0]
                    eyf = Rf[:, 1]
                    ezf = Rf[:, 2]

                    ax.plot(
                        [pf[0] - exf[0] * quad_arm_length * 0.8, pf[0] + exf[0] * quad_arm_length * 0.8],
                        [pf[1] - exf[1] * quad_arm_length * 0.8, pf[1] + exf[1] * quad_arm_length * 0.8],
                        [pf[2] - exf[2] * quad_arm_length * 0.8, pf[2] + exf[2] * quad_arm_length * 0.8],
                        color="yellow",
                        linewidth=3.0,
                        alpha=1.0,
                        label="Final quad",
                        zorder=8,
                    )
                    ax.plot(
                        [pf[0] - eyf[0] * quad_arm_length * 0.8, pf[0] + eyf[0] * quad_arm_length * 0.8],
                        [pf[1] - eyf[1] * quad_arm_length * 0.8, pf[1] + eyf[1] * quad_arm_length * 0.8],
                        [pf[2] - eyf[2] * quad_arm_length * 0.8, pf[2] + eyf[2] * quad_arm_length * 0.8],
                        color="yellow",
                        linewidth=3.0,
                        alpha=1.0,
                        zorder=8,
                    )
                    ax.quiver(
                        pf[0], pf[1], pf[2],
                        exf[0], exf[1], exf[2],
                        length=attitude_axis_scale * 1.8,
                        color="red",
                        alpha=1.0,
                        linewidth=2.0,
                    )
                    ax.quiver(
                        pf[0], pf[1], pf[2],
                        eyf[0], eyf[1], eyf[2],
                        length=attitude_axis_scale * 1.8,
                        color="lime",
                        alpha=1.0,
                        linewidth=2.0,
                    )
                    ax.quiver(
                        pf[0], pf[1], pf[2],
                        ezf[0], ezf[1], ezf[2],
                        length=attitude_axis_scale * 1.8,
                        color="dodgerblue",
                        alpha=1.0,
                        linewidth=2.0,
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
    elif terrain_type == "mountains":
        terrain = next((o for o in obstacles if isinstance(o, HeightFieldTerrain)), None)
        cylinders = [o for o in obstacles if hasattr(o, "radius")]

        if terrain is not None:
            X, Y = np.meshgrid(terrain.xs, terrain.ys, indexing="ij")
            ax.plot_surface(
                X,
                Y,
                terrain.heights,
                cmap=terrain_cmap,
                linewidth=0.0,
                antialiased=True,
                alpha=terrain_alpha,
                zorder=1,
            )

        if cylinders:
            obs_x = [float(c.center[0]) for c in cylinders]
            obs_y = [float(c.center[1]) for c in cylinders]
            obs_z = [float(c.center[2]) + float(getattr(c, "height", 0.0)) / 2.0 for c in cylinders]
            ax.scatter(obs_x, obs_y, obs_z, color="gray", alpha=0.6, label="Rocks", zorder=2)
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
    title_backend = f" [{str(backend_name).upper()}]" if backend_name else ""
    ax.set_title(f"{title_terrain} path planner with {title_planner}{title_backend}")

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
