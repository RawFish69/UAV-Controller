"""Position-level controllers for the standalone simulator.

These operate on a simple point-mass model:
  x_dot = v, v_dot = a_cmd
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are


def _as_vector(value: np.ndarray) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if not np.all(np.isfinite(arr)):
        raise ValueError("position control received non-finite state/target")
    return arr


def pid_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    kp: float,
    kd: float,
) -> np.ndarray:
    """Compute acceleration command using simple PD on position."""
    pos = _as_vector(pos)
    vel = _as_vector(vel)
    target = _as_vector(target)

    pos_error = target - pos
    vel_error = -vel
    acc_cmd = kp * pos_error + kd * vel_error
    return acc_cmd


def lqr_gain_double_integrator(q_pos: float, q_vel: float, r_acc: float) -> np.ndarray:
    """Compute continuous-time LQR gain for a 1D double integrator."""
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    Q = np.diag([q_pos, q_vel])
    R = np.array([[r_acc]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


def lqr_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    q_pos: float = 10.0,
    q_vel: float = 2.0,
    r_acc: float = 1.0,
) -> np.ndarray:
    """LQR state-feedback for each axis of a double integrator."""
    pos = _as_vector(pos)
    vel = _as_vector(vel)
    target = _as_vector(target)

    x_err = np.stack([pos - target, vel], axis=-1)
    K = lqr_gain_double_integrator(q_pos, q_vel, r_acc)
    acc_cmd = -np.einsum("ij,bj->bi", K, x_err).squeeze(-1)
    return acc_cmd


def mpc_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    q_pos: float,
    q_vel: float,
    r_acc: float,
    dt: float = 0.1,
    horizon: int = 10,
) -> np.ndarray:
    """Finite-horizon discrete LQ controller (a real receding-horizon MPC).

    This is no longer an alias for LQR: it solves the discrete-time Riccati
    recursion over ``horizon`` steps and applies the first-step optimal gain.
    """
    pos = _as_vector(pos)
    vel = _as_vector(vel)
    target = _as_vector(target)

    dt = max(float(dt), 1e-4)
    horizon = max(int(horizon), 1)
    A = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
    B = np.array([[0.5 * dt * dt], [dt]], dtype=float)
    Q = np.diag([float(q_pos), float(q_vel)])
    R = np.array([[float(r_acc)]], dtype=float)

    P = Q
    K = np.zeros((1, 2), dtype=float)
    for _ in range(horizon):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P = Q + A.T @ P @ A - A.T @ P @ B @ K

    x_err = np.stack([pos - target, vel], axis=-1)
    acc_cmd = -np.einsum("ij,bj->bi", K, x_err).squeeze(-1)
    return acc_cmd


def mppi_position_control(
    pos: np.ndarray,
    vel: np.ndarray,
    target: np.ndarray,
    *,
    dt: float = 0.1,
    horizon: int = 12,
    samples: int = 300,
    q_pos: float = 10.0,
    q_vel: float = 1.0,
    r_acc: float = 0.05,
    noise_std: float = 2.0,
    temperature: float = 1.0,
    seed: int | None = None,
) -> np.ndarray:
    """Sample-based MPPI for the double-integrator point-mass model.

    Returns the first acceleration command from the weighted-average control
    sequence. This is intentionally small and dependency-free.
    """
    pos = _as_vector(pos)
    vel = _as_vector(vel)
    target = _as_vector(target)
    dt = max(float(dt), 1e-4)
    horizon = max(int(horizon), 1)
    samples = max(int(samples), 1)
    rng = np.random.default_rng(seed)

    # Rollout samples: each is a [horizon, 3] acceleration sequence.
    noise = rng.normal(0.0, float(noise_std), size=(samples, horizon, 3))

    # Base zero-acceleration rollout cost, used to scale weights.
    pos_err = pos - target
    costs = np.empty(samples, dtype=float)
    first_actions = np.empty((samples, 3), dtype=float)

    for i in range(samples):
        p = pos_err.copy()
        v = vel.copy()
        total = 0.0
        for k in range(horizon):
            a = noise[i, k]
            if k == 0:
                first_actions[i] = a
            total += float(q_pos) * float(np.dot(p, p)) + float(q_vel) * float(np.dot(v, v))
            total += float(r_acc) * float(np.dot(a, a))
            v = v + a * dt
            p = p + v * dt
        costs[i] = total

    min_cost = float(np.min(costs))
    weights = np.exp(-(costs - min_cost) / max(float(temperature), 1e-6))
    weights /= max(float(np.sum(weights)), 1e-12)
    acc_cmd = np.sum(first_actions * weights[:, None], axis=0)
    return np.clip(acc_cmd, -20.0, 20.0)


__all__ = [
    "pid_position_control",
    "lqr_gain_double_integrator",
    "lqr_position_control",
    "mpc_position_control",
    "mppi_position_control",
]
