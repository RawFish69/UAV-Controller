from __future__ import annotations

import unittest
from pathlib import Path

from sim_py.core.config import NormalizedSimConfig
from sim_py.core.runner import run_simulation


class TestRunnerIntegration(unittest.TestCase):
    def _base_cfg(self, controller_name: str) -> NormalizedSimConfig:
        return NormalizedSimConfig(
            sim_config_path=Path("sim_py/sim_config.yaml"),
            terrain_override="plains",
            terrain_config_path=None,
            controller_name=controller_name,
            backend_name="pointmass",
            dt=0.05,
            sim_time=0.3,
            path_cfg={
                "planner_type": "straight",
                "start_relative_x": 0.1,
                "start_relative_y": 0.1,
                "start_relative_z": 0.1,
                "end_relative_x": 0.2,
                "end_relative_y": 0.2,
                "end_relative_z": 0.2,
                "start_xy_offset_range": 0.0,
                "end_xy_offset_range": 0.0,
                "collision_inflation": 0.0,
                "terrain_clearance": 0.0,
            },
            controller_cfg={
                "acc_max": 20.0,
                "pid": {"kp": 0.8, "kd": 1.2},
                "lqr": {"q_pos": 10.0, "q_vel": 2.0, "r_acc": 1.0},
                "mpc": {"q_pos": 8.0, "q_vel": 2.0, "r_acc": 10.0},
            },
            visual_cfg={
                "forest_density_scale": 1.0,
                "tree_height_scale": 1.0,
                "height_ratio": 1.2,
            },
            simulation_cfg={"backend": "pointmass", "seed": 123},
            raw_cfg={},
            seed=123,
        )

    def test_default_run_with_pointmass(self) -> None:
        sim_log = run_simulation(self._base_cfg("pid"))
        self.assertGreaterEqual(sim_log.trajectory.shape[0], 2)
        self.assertEqual(sim_log.trajectory.shape[1], 3)

    def test_lqr_run_with_pointmass(self) -> None:
        sim_log = run_simulation(self._base_cfg("lqr"))
        self.assertGreaterEqual(sim_log.trajectory.shape[0], 2)
        self.assertEqual(sim_log.trajectory.shape[1], 3)


if __name__ == "__main__":
    unittest.main()
