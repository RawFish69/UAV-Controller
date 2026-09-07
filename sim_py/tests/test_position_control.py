from __future__ import annotations

import numpy as np

from aerial_kit.controllers.position import (
    lqr_position_control,
    mpc_position_control,
    mppi_position_control,
    pid_position_control,
)


def test_pid_position_control_moves_toward_target():
    pos = np.array([0.0, 0.0, 0.0])
    vel = np.array([0.0, 0.0, 0.0])
    target = np.array([1.0, -1.0, 0.5])
    acc = pid_position_control(pos, vel, target, kp=0.8, kd=1.2)
    assert np.all(np.isfinite(acc))
    assert np.dot(acc, target) > 0.0


def test_lqr_position_control_moves_toward_target():
    pos = np.array([1.0, -2.0, 0.0])
    vel = np.array([0.0, 0.0, 0.0])
    target = np.zeros(3)
    acc = lqr_position_control(pos, vel, target, q_pos=10.0, q_vel=2.0, r_acc=1.0)
    assert np.all(np.isfinite(acc))
    assert np.dot(acc, -pos) > 0.0


def test_mpc_position_control_is_finite_and_stable():
    pos = np.array([1.0, 1.0, 1.0])
    vel = np.array([0.1, -0.1, 0.0])
    target = np.zeros(3)
    acc = mpc_position_control(
        pos, vel, target, q_pos=8.0, q_vel=2.0, r_acc=10.0, dt=0.1, horizon=8
    )
    assert np.all(np.isfinite(acc))


def test_mppi_position_control_is_finite_and_reduces_error():
    pos = np.array([1.0, -1.0, 0.0])
    vel = np.array([0.0, 0.0, 0.0])
    target = np.zeros(3)
    acc = mppi_position_control(
        pos,
        vel,
        target,
        dt=0.1,
        horizon=6,
        samples=80,
        q_pos=10.0,
        q_vel=1.0,
        r_acc=0.05,
        noise_std=2.0,
        temperature=1.0,
        seed=0,
    )
    assert np.all(np.isfinite(acc))
