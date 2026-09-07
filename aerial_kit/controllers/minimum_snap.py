"""Small minimum-snap polynomial trajectory utilities."""

from __future__ import annotations

import numpy as np


def _poly_derivatives(coeffs: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate position/velocity/acceleration for polynomial coefficients."""
    order = coeffs.shape[1] - 1
    powers = np.arange(order + 1, dtype=float)
    tp = np.power(t[:, None], powers)
    pos = tp @ coeffs.T
    vel = (powers[1:] * np.power(t[:, None], powers[1:] - 1.0)) @ coeffs[:, 1:].T
    acc = (powers[2:] * (powers[2:] - 1.0) * np.power(t[:, None], powers[2:] - 2.0)) @ coeffs[:, 2:].T
    return pos, vel, acc


def minimum_snap_trajectory(
    waypoints: np.ndarray,
    times: np.ndarray | None = None,
    polynomial_order: int = 7,
    snap_weight: float = 1e-4,
    num_samples: int = 100,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit a smooth polynomial through waypoints and sample it.

    This is a dependency-free regularized polynomial fit: it minimizes position
    error plus a penalty on high-order coefficients (a proxy for snap) rather
    than solving a constrained QP. Returns ``(positions, velocities,
    accelerations)``, each shaped ``(num_samples, 3)``.
    """
    waypoints = np.asarray(waypoints, dtype=float).reshape(-1, 3)
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")

    if times is None:
        times = np.linspace(0.0, 1.0, len(waypoints))
    times = np.asarray(times, dtype=float).reshape(-1)
    if times.shape != (len(waypoints),):
        raise ValueError("times must match the number of waypoints")

    order = max(int(polynomial_order), len(waypoints) - 1)
    V = np.power(times[:, None], np.arange(order + 1, dtype=float))
    D = np.diag([float(snap_weight) if i >= 4 else 0.0 for i in range(order + 1)])
    A = V.T @ V + D
    b = V.T @ waypoints
    coeffs = np.linalg.solve(A, b).T  # (3, order+1)

    sample_t = np.linspace(float(times[0]), float(times[-1]), max(int(num_samples), 2))
    pos, vel, acc = _poly_derivatives(coeffs, sample_t)
    return pos, vel, acc


__all__ = ["minimum_snap_trajectory"]
