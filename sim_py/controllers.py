"""
Position-level controllers for the standalone simulator.

These operate on a simple point-mass model:
  x_dot = v, v_dot = a_cmd

Controllers provided:
- PID: PD control on position/velocity error
- LQR: continuous-time LQR on double integrator
- MPC: simple finite-horizon quadratic MPC (implemented as LQR with
       different tuning for now, suitable for comparison).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are


def pid_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    kp: float,
    kd: float,
) -> np.ndarray:
    """Compute acceleration command using simple PD on position."""
    pos = np.asarray(pos, dtype=float)
    vel = np.asarray(vel, dtype=float)
    target = np.asarray(target, dtype=float)

    pos_error = target - pos
    vel_error = -vel
    acc_cmd = kp * pos_error + kd * vel_error
    return acc_cmd


def lqr_gain_double_integrator(q_pos: float, q_vel: float, r_acc: float) -> np.ndarray:
    """
    Compute continuous-time LQR gain for 1D double integrator:
        x = [pos, vel], u = acc.
    """
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    Q = np.diag([q_pos, q_vel])
    R = np.array([[r_acc]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P  # shape (1, 2)
    return K


def lqr_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    q_pos: float = 10.0,
    q_vel: float = 2.0,
    r_acc: float = 1.0,
) -> np.ndarray:
    """
    LQR state-feedback for each axis of a double integrator.
    """
    pos = np.asarray(pos, dtype=float)
    vel = np.asarray(vel, dtype=float)
    target = np.asarray(target, dtype=float)

    # State error for each axis: [pos_error, vel]
    # We treat the origin as the desired error (0, 0).
    x_err = np.stack([pos - target, vel], axis=-1)  # shape (3, 2)
    K = lqr_gain_double_integrator(q_pos, q_vel, r_acc)  # (1, 2)
    acc_cmd = -np.einsum("ij,bj->bi", K, x_err).squeeze(-1)
    return acc_cmd


def mpc_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    q_pos: float,
    q_vel: float,
    r_acc: float,
) -> np.ndarray:
    """
    Very lightweight MPC-like controller.

    For now this uses LQR-style state feedback but with configurable
    weighting to mimic an unconstrained linear MPC.
    """
    return lqr_position_control(pos, vel, target, q_pos=q_pos, q_vel=q_vel, r_acc=r_acc)




