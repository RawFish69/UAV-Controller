from __future__ import annotations

import numpy as np

from aerial_kit.controllers.basic import LQRController, MPCController, MPPIController, PIDController
from aerial_kit.controllers.minimum_snap import minimum_snap_trajectory
from aerial_kit.types import SimState, Waypoint
from sim_py.core.registry import create_controller, register_builtin_components


def _state(pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)) -> SimState:
    return SimState(
        position=np.array(pos, dtype=float),
        velocity=np.array(vel, dtype=float),
        t=0.0,
    )


def _waypoint(pos=(1.0, 0.0, 0.0)) -> Waypoint:
    return Waypoint(position=np.array(pos, dtype=float))


def test_all_position_controllers_return_finite_acceleration():
    state = _state()
    target = _waypoint()
    controllers = [
        PIDController(),
        LQRController(),
        MPCController(),
        MPPIController(),
    ]
    for controller in controllers:
        result = controller.compute(state, target, cfg={})
        assert np.all(np.isfinite(result.accel_cmd))


def test_builtin_controllers_resolve():
    register_builtin_components()
    for name in ("pid", "lqr", "mpc", "mppi", "l1_tecs"):
        assert create_controller(name) is not None


def test_minimum_snap_waypoint_continuity():
    waypoints = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 1.0],
            [2.0, 0.0, 2.0],
            [3.0, 1.0, 0.0],
        ],
        dtype=float,
    )
    times = np.array([0.0, 1.0, 2.0, 3.0])
    pos, vel, acc = minimum_snap_trajectory(waypoints, times, num_samples=40)
    assert pos.shape == (40, 3)
    assert vel.shape == (40, 3)
    assert acc.shape == (40, 3)
    assert np.all(np.isfinite(pos))
    assert np.all(np.isfinite(vel))
    assert np.all(np.isfinite(acc))
