"""
Entry point for the standalone Python UAV simulator.

Usage (from repo root):

    pip install -r sim_py/requirements.txt
    python -m sim_py.run_sim --terrain forest --controller pid
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import numpy as np
import yaml

from .controllers import lqr_position_control, mpc_position_control, pid_position_control
from .dynamics import PointMassDynamics, PointMassParams
from .planner import default_plan
from .terrain_wrapper import TerrainConfig, generate_terrain, is_point_in_collision, load_terrain_config
from .visualizer import plot_simulation

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone UAV simulator (no ROS2).")
    parser.add_argument(
        "--terrain",
        type=str,
        choices=["forest", "mountains", "plains"],
        default=None,
        help="Override terrain type from YAML.",
    )
    parser.add_argument(
        "--terrain-config",
        type=str,
        default=None,
        help="Path to terrain_params.yaml (defaults to ROS2 config).",
    )
    parser.add_argument(
        "--controller",
        type=str,
        choices=["pid", "lqr", "mpc"],
        default="pid",
        help="Controller type to use.",
    )
    parser.add_argument(
        "--sim-time",
        type=float,
        default=None,
        help="Total simulation time [s] (overrides sim_config.yaml).",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Simulation time step [s] (overrides sim_config.yaml).",
    )
    parser.add_argument(
        "--sim-config",
        type=str,
        default="sim_py/sim_config.yaml",
        help="Path to sim_config.yaml.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    logger.info("=" * 60)
    logger.info("Starting UAV Simulation")
    logger.info("=" * 60)

    # Load simulator configuration directly from YAML
    cfg_path = Path(args.sim_config)
    if cfg_path.exists():
        logger.info(f"Loading config from: {cfg_path}")
        with cfg_path.open("r") as f:
            sim_cfg = yaml.safe_load(f) or {}
    else:
        logger.warning(f"Config file not found: {cfg_path}, using defaults")
        sim_cfg = {}

    path_cfg = sim_cfg.get("path", {})
    ctrl_cfg = sim_cfg.get("controller", {})
    vis_cfg = sim_cfg.get("visual", {})

    # Terrain configuration and generation
    cfg: TerrainConfig
    if args.terrain_config is not None:
        cfg = load_terrain_config(Path(args.terrain_config))
    else:
        cfg = load_terrain_config()

    if args.terrain is not None:
        cfg.terrain_type = args.terrain  # Override from CLI

    logger.info(f"Terrain type: {cfg.terrain_type}")
    logger.info(f"Space dimensions: {cfg.space_dim}")

    obstacles = generate_terrain(
        cfg,
        forest_density_scale=float(vis_cfg.get("forest_density_scale", 1.0)),
        tree_height_scale=float(vis_cfg.get("tree_height_scale", 1.0)),
    )
    logger.info(f"Generated {len(obstacles)} obstacles")

    # Compute max allowed flight altitude: cannot fly above tallest tree
    max_tree_top = 0.0
    for o in obstacles:
        c = o.center
        top = float(c[2])
        if hasattr(o, "height"):
            top += float(getattr(o, "height"))
        elif hasattr(o, "size"):
            top += float(getattr(o, "size")[2])
        if top > max_tree_top:
            max_tree_top = top

    if cfg.terrain_type == "forest" and max_tree_top > 0.0:
        height_ratio = float(vis_cfg.get("height_ratio", 1.2))
        cfg.space_dim = cfg.space_dim.copy()
        cfg.space_dim[2] = max_tree_top * height_ratio
        logger.info(
            f"Adjusted space Z to {cfg.space_dim[2]:.2f} m "
            f"(ratio {height_ratio:.2f} * tallest tree)"
        )

    max_z_allowed = min(max_tree_top, float(cfg.space_dim[2]))
    logger.info(f"Max allowed altitude: {max_z_allowed:.2f} m (tallest obstacle: {max_tree_top:.2f} m)")

    # Compute nominal start/goal positions from config
    # Values are treated as fractions of space_dim (0.0-1.0 = within map, >1.0 = outside map)
    sx_rel = float(path_cfg.get("start_relative_x", 0.02))
    sy_rel = float(path_cfg.get("start_relative_y", 0.02))
    sz_rel = float(path_cfg.get("start_relative_z", 0.05))
    gx_rel = float(path_cfg.get("end_relative_x", 0.98))
    gy_rel = float(path_cfg.get("end_relative_y", 0.98))
    gz_raw = path_cfg.get("end_relative_z", 0.9)
    auto_goal_z = isinstance(gz_raw, str) and gz_raw.lower() in {"auto", "auto_tree", "tree"}

    sx = sx_rel * float(cfg.space_dim[0])
    sy = sy_rel * float(cfg.space_dim[1])
    sz = sz_rel * float(cfg.space_dim[2])
    gx = gx_rel * float(cfg.space_dim[0])
    gy = gy_rel * float(cfg.space_dim[1])
    if auto_goal_z:
        gz = float(np.random.uniform(0.0, max_z_allowed))
        gz_rel = gz / float(cfg.space_dim[2]) if float(cfg.space_dim[2]) > 0 else 0.0
        logger.info(f"Goal Z set randomly in [0, {max_z_allowed:.2f}] m")
    else:
        gz_rel = float(gz_raw)
        gz = gz_rel * float(cfg.space_dim[2])
    path_start = np.array([sx, sy, sz], dtype=float)
    path_goal = np.array([gx, gy, gz], dtype=float)

    # Normalize path_cfg to avoid passing non-numeric goal Z to planner
    path_cfg = dict(path_cfg)
    path_cfg["end_relative_z"] = gz_rel

    # Warn if goal is outside map bounds
    if gx > cfg.space_dim[0] or gy > cfg.space_dim[1] or gz > cfg.space_dim[2]:
        logger.warning(
            f"Goal position [{gx:.1f}, {gy:.1f}, {gz:.1f}] is outside map bounds "
            f"[{cfg.space_dim[0]:.1f}, {cfg.space_dim[1]:.1f}, {cfg.space_dim[2]:.1f}]"
        )

    logger.info(f"Start relative: [{sx_rel:.3f}, {sy_rel:.3f}, {sz_rel:.3f}]")
    logger.info(f"Goal relative: [{gx_rel:.3f}, {gy_rel:.3f}, {gz_rel:.3f}]")
    logger.info(f"Space dimensions: [{cfg.space_dim[0]:.1f}, {cfg.space_dim[1]:.1f}, {cfg.space_dim[2]:.1f}]")
    logger.info(f"Start position (absolute): [{path_start[0]:.2f}, {path_start[1]:.2f}, {path_start[2]:.2f}]")
    logger.info(f"Goal position (absolute): [{path_goal[0]:.2f}, {path_goal[1]:.2f}, {path_goal[2]:.2f}]")

    # Planner: simple straight-line path using these start/goal points
    planner_type = str(path_cfg.get("planner_type", "straight")).lower()
    logger.info(f"Planning path using: {planner_type.upper()}")
    waypoints = default_plan(cfg.space_dim, cfg.start_pos, obstacles, path_cfg=path_cfg)
    logger.info(f"Planned path with {len(waypoints)} waypoints")

    # Verify planned path for collisions
    inflation = float(path_cfg.get("collision_inflation", 0.5))
    path_collisions = 0
    for i, wp in enumerate(waypoints):
        if is_point_in_collision(wp.position, obstacles, inflation=inflation):
            path_collisions += 1
            if path_collisions == 1:
                logger.warning(f"  Planned path has collisions! Waypoint {i} at [{wp.position[0]:.2f}, {wp.position[1]:.2f}, {wp.position[2]:.2f}]")
    if path_collisions > 0:
        logger.warning(f"  WARNING: {path_collisions}/{len(waypoints)} waypoints are in collision!")
    else:
        logger.info(f"  Planned path verified: all waypoints collision-free ✓")

    # Dynamics: simple point-mass model, initialized at path start
    pm_params = PointMassParams()
    dynamics = PointMassDynamics(pm_params)
    dynamics.position = path_start.copy()
    dynamics.velocity[:] = 0.0

    t = 0.0
    # dt / sim_time preference: CLI overrides YAML if provided
    dt = float(args.dt) if args.dt is not None else float(ctrl_cfg.get("dt", 0.01))
    max_t = (
        float(args.sim_time) if args.sim_time is not None else float(ctrl_cfg.get("sim_time", 20.0))
    )

    controller_type = args.controller or ctrl_cfg.get("controller_type", "pid")
    logger.info(f"Controller: {controller_type.upper()}")
    logger.info(f"Simulation time: {max_t:.1f} s, dt: {dt:.3f} s")
    logger.info("-" * 60)
    logger.info("Starting simulation loop...")

    traj_positions = [dynamics.position.copy()]

    wp_idx = 0
    last_log_time = 0.0
    log_interval = 5.0  # Log every 5 seconds
    collisions_detected = 0

    while t < max_t and wp_idx < len(waypoints):
        state = dynamics.state
        pos = state["position"]

        # Advance to next waypoint once close enough
        target = waypoints[wp_idx].position
        dist_to_target = np.linalg.norm(target - pos)
        if dist_to_target < 1.0 and wp_idx < len(waypoints) - 1:
            wp_idx += 1
            target = waypoints[wp_idx].position
            logger.info(f"  t={t:.2f}s: Reached waypoint {wp_idx-1}/{len(waypoints)-1}, advancing to next")

        vel = state["velocity"]

        # Check for collisions with obstacles
        if is_point_in_collision(pos, obstacles, inflation=0.0):
            collisions_detected += 1
            if collisions_detected == 1:  # Log first collision
                logger.warning(f"  t={t:.2f}s: COLLISION DETECTED at [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

        # Position-level control: compute acceleration command based on
        # selected controller type (CLI overrides config if provided).

        if controller_type == "pid":
            pid_cfg = ctrl_cfg.get("pid", {})
            kp = float(pid_cfg.get("kp", 0.8))
            kd = float(pid_cfg.get("kd", 1.2))
            acc_cmd = pid_position_control(pos, vel, target, kp=kp, kd=kd)
        elif controller_type == "lqr":
            lqr_cfg = ctrl_cfg.get("lqr", {})
            acc_cmd = lqr_position_control(
                pos,
                vel,
                target,
                q_pos=float(lqr_cfg.get("q_pos", 10.0)),
                q_vel=float(lqr_cfg.get("q_vel", 2.0)),
                r_acc=float(lqr_cfg.get("r_acc", 1.0)),
            )
        else:  # "mpc"
            mpc_cfg = ctrl_cfg.get("mpc", {})
            acc_cmd = mpc_position_control(
                pos,
                vel,
                target,
                q_pos=float(mpc_cfg.get("q_pos", 8.0)),
                q_vel=float(mpc_cfg.get("q_vel", 2.0)),
                r_acc=float(mpc_cfg.get("r_acc", 10.0)),
            )

        # Limit acceleration to something reasonable
        acc_mag = np.linalg.norm(acc_cmd)
        acc_max = float(ctrl_cfg.get("acc_max", 20.0))  # [m/s^2]
        if acc_mag > acc_max:
            acc_cmd = acc_cmd * (acc_max / (acc_mag + 1e-6))

        dynamics.step(acc_cmd, dt)

        # Keep the simulated UAV inside the configured terrain box and under tallest tree
        dynamics.position = np.clip(
            dynamics.position,
            np.array([0.0, 0.0, 0.0]),
            np.array(
                [
                    float(cfg.space_dim[0]),
                    float(cfg.space_dim[1]),
                    max_z_allowed,
                ]
            ),
        )

        traj_positions.append(dynamics.position.copy())
        t += dt

        # Periodic logging
        if t - last_log_time >= log_interval:
            dist_to_goal = np.linalg.norm(waypoints[-1].position - pos)
            logger.info(
                f"  t={t:.1f}s: pos=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], "
                f"vel={np.linalg.norm(vel):.2f} m/s, "
                f"waypoint {wp_idx}/{len(waypoints)-1}, "
                f"dist_to_goal={dist_to_goal:.1f} m"
            )
            last_log_time = t

    trajectory = np.vstack(traj_positions)

    logger.info("-" * 60)
    logger.info("Simulation completed")
    logger.info(f"  Final time: {t:.2f} s")
    logger.info(f"  Final position: [{trajectory[-1, 0]:.2f}, {trajectory[-1, 1]:.2f}, {trajectory[-1, 2]:.2f}]")
    logger.info(f"  Final waypoint reached: {wp_idx}/{len(waypoints)-1}")
    dist_to_goal = np.linalg.norm(waypoints[-1].position - trajectory[-1])
    logger.info(f"  Distance to goal: {dist_to_goal:.2f} m")
    if collisions_detected > 0:
        logger.warning(f"  COLLISIONS DETECTED: {collisions_detected} (path may intersect obstacles!)")
    else:
        logger.info("  No collisions detected ✓")
    logger.info("=" * 60)

    # Extract planned waypoint positions for visualization
    planned_waypoints = np.array([wp.position for wp in waypoints])

    plot_simulation(
        trajectory,
        obstacles,
        space_dim=cfg.space_dim,
        terrain_type=cfg.terrain_type,
        visual_cfg=vis_cfg,
        planned_waypoints=planned_waypoints,
        goal_position=path_goal,
        planner_type=planner_type,
        show=True,
    )


if __name__ == "__main__":
    main()

