"""Built-in controller implementations for the simulation framework."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..interfaces import Controller
from ..types import ControlTarget, SimState, Waypoint
from .position import (
    lqr_position_control,
    mpc_position_control,
    mppi_position_control,
    pid_position_control,
)


class PIDController(Controller):
    """PD position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        pid_cfg = dict(ctrl_cfg.get("pid", {}) or {})
        kp = float(pid_cfg.get("kp", 0.8))
        kd = float(pid_cfg.get("kd", 1.2))
        max_acc_mps2 = float(pid_cfg.get("max_acc_mps2", 20.0))

        acc_cmd = pid_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            kp=kp,
            kd=kd,
        )
        acc_cmd = np.clip(np.asarray(acc_cmd, dtype=float), -max_acc_mps2, max_acc_mps2)
        return ControlTarget(accel_cmd=acc_cmd, metadata={"controller": "pid"})


class LQRController(Controller):
    """LQR position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        lqr_cfg = dict(ctrl_cfg.get("lqr", {}) or {})
        max_acc_mps2 = float(lqr_cfg.get("max_acc_mps2", 20.0))
        acc_cmd = lqr_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            q_pos=float(lqr_cfg.get("q_pos", 10.0)),
            q_vel=float(lqr_cfg.get("q_vel", 2.0)),
            r_acc=float(lqr_cfg.get("r_acc", 1.0)),
        )
        acc_cmd = np.clip(np.asarray(acc_cmd, dtype=float), -max_acc_mps2, max_acc_mps2)
        return ControlTarget(accel_cmd=acc_cmd, metadata={"controller": "lqr"})


class MPCController(Controller):
    """MPC-like position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        mpc_cfg = dict(ctrl_cfg.get("mpc", {}) or {})
        max_acc_mps2 = float(mpc_cfg.get("max_acc_mps2", 20.0))
        acc_cmd = mpc_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            q_pos=float(mpc_cfg.get("q_pos", 8.0)),
            q_vel=float(mpc_cfg.get("q_vel", 2.0)),
            r_acc=float(mpc_cfg.get("r_acc", 10.0)),
            dt=float(mpc_cfg.get("dt", 0.1)),
            horizon=int(mpc_cfg.get("horizon", 10)),
        )
        acc_cmd = np.clip(np.asarray(acc_cmd, dtype=float), -max_acc_mps2, max_acc_mps2)
        return ControlTarget(accel_cmd=acc_cmd, metadata={"controller": "mpc"})


class MPPIController(Controller):
    """MPPI position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        mppi_cfg = dict(ctrl_cfg.get("mppi", {}) or {})
        max_acc_mps2 = float(mppi_cfg.get("max_acc_mps2", 20.0))
        acc_cmd = mppi_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            dt=float(mppi_cfg.get("dt", 0.1)),
            horizon=int(mppi_cfg.get("horizon", 12)),
            samples=int(mppi_cfg.get("samples", 300)),
            q_pos=float(mppi_cfg.get("q_pos", 10.0)),
            q_vel=float(mppi_cfg.get("q_vel", 1.0)),
            r_acc=float(mppi_cfg.get("r_acc", 0.05)),
            noise_std=float(mppi_cfg.get("noise_std", 2.0)),
            temperature=float(mppi_cfg.get("temperature", 1.0)),
            seed=int(mppi_cfg.get("seed", 0)) if "seed" in mppi_cfg else None,
        )
        acc_cmd = np.clip(np.asarray(acc_cmd, dtype=float), -max_acc_mps2, max_acc_mps2)
        return ControlTarget(accel_cmd=acc_cmd, metadata={"controller": "mppi"})
