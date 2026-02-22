"""CLI argument parsing for the standalone simulator."""

from __future__ import annotations

import argparse


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
        choices=["pid", "lqr", "mpc", "teleop"],
        default="pid",
        help="Controller type to use.",
    )
    parser.add_argument(
        "--backend",
        type=str,
        choices=["pointmass", "rotorpy"],
        default=None,
        help="Dynamics backend to use (default: pointmass or simulation.backend).",
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
