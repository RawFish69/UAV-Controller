from __future__ import annotations

import importlib.util
import unittest

import numpy as np

from sim_py.backends.pointmass_backend import PointMassBackend
from sim_py.backends.rotorpy_backend import RotorPyBackend
from sim_py.core.types import ControlTarget, SimState


class TestBackends(unittest.TestCase):
    def test_pointmass_backend_interface(self) -> None:
        backend = PointMassBackend()
        backend.reset(
            initial_state=SimState(
                position=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity=np.zeros(3, dtype=float),
                t=0.0,
            ),
            world={},
            cfg={"simulation": {}},
        )
        backend.step(ControlTarget(accel_cmd=np.array([0.1, 0.0, 0.0], dtype=float)), dt=0.1)
        s = backend.state()
        self.assertGreater(s.t, 0.0)
        self.assertEqual(s.position.shape, (3,))
        self.assertEqual(s.velocity.shape, (3,))

    def test_rotorpy_backend_import_guard_or_step(self) -> None:
        backend = RotorPyBackend()
        has_rotorpy = importlib.util.find_spec("rotorpy") is not None

        if not has_rotorpy:
            with self.assertRaises(RuntimeError) as e:
                backend.reset(
                    initial_state=SimState(
                        position=np.array([0.0, 0.0, 1.0], dtype=float),
                        velocity=np.zeros(3, dtype=float),
                        t=0.0,
                    ),
                    world={},
                    cfg={"simulation": {}},
                )
            self.assertIn("requirements-rotorpy.txt", str(e.exception))
            return

        backend.reset(
            initial_state=SimState(
                position=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity=np.zeros(3, dtype=float),
                t=0.0,
            ),
            world={},
            cfg={"simulation": {}},
        )
        backend.step(ControlTarget(accel_cmd=np.zeros(3, dtype=float)), dt=0.01)
        s = backend.state()
        self.assertGreaterEqual(s.t, 0.01)
        self.assertEqual(s.position.shape, (3,))


if __name__ == "__main__":
    unittest.main()
