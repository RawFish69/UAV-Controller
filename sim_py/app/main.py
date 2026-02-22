"""Main entrypoint for standalone simulator app."""

from __future__ import annotations

import logging

from ..core.config import normalize_sim_config
from ..core.runner import run_simulation
from ..visualizer import plot_simulation
from .cli import parse_args
from .teleop import run_teleop_session


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


def main() -> None:
    args = parse_args()
    cfg_norm = normalize_sim_config(args)

    logger.info("=" * 60)
    logger.info("Starting UAV Simulation")
    logger.info("=" * 60)
    logger.info(f"Loading config from: {cfg_norm.sim_config_path}")

    if str(cfg_norm.controller_name).lower() == "teleop":
        try:
            run_teleop_session(cfg_norm)
        except RuntimeError as exc:
            msg = str(exc)
            if "requirements-rotorpy.txt" in msg:
                logger.error(msg)
                raise SystemExit(1)
            raise
        return

    try:
        sim_log = run_simulation(cfg_norm)
    except RuntimeError as exc:
        msg = str(exc)
        if "requirements-rotorpy.txt" in msg:
            logger.error(msg)
            raise SystemExit(1)
        raise

    plot_simulation(
        sim_log.trajectory,
        sim_log.obstacles,
        space_dim=sim_log.space_dim,
        terrain_type=sim_log.terrain_type,
        visual_cfg=sim_log.visual_cfg,
        planned_waypoints=sim_log.planned_waypoints,
        goal_position=sim_log.goal_position,
        planner_type=sim_log.planner_type,
        attitude_quats=sim_log.attitude_quats,
        backend_name=sim_log.backend_name,
        show=True,
    )


if __name__ == "__main__":
    main()
