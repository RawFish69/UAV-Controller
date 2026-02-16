from __future__ import annotations

import argparse
import tempfile
import unittest
from pathlib import Path

import yaml

from sim_py.core.config import normalize_sim_config


class TestConfigNormalization(unittest.TestCase):
    def test_legacy_defaults_and_overrides(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            cfg_path = Path(d) / "sim.yaml"
            cfg_path.write_text(
                yaml.safe_dump(
                    {
                        "controller": {
                            "controller_type": "lqr",
                            "dt": 0.2,
                            "sim_time": 99.0,
                        },
                        "simulation": {
                            "backend": "rotorpy",
                            "seed": 7,
                        },
                    }
                ),
                encoding="utf-8",
            )

            args = argparse.Namespace(
                terrain=None,
                terrain_config=None,
                controller="pid",  # keep legacy CLI behavior
                backend=None,
                sim_time=None,
                dt=None,
                sim_config=str(cfg_path),
            )

            cfg = normalize_sim_config(args)
            self.assertEqual(cfg.controller_name, "pid")
            self.assertEqual(cfg.backend_name, "rotorpy")
            self.assertAlmostEqual(cfg.dt, 0.2)
            self.assertAlmostEqual(cfg.sim_time, 99.0)
            self.assertEqual(cfg.seed, 7)

    def test_cli_dt_and_sim_time_override_yaml(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            cfg_path = Path(d) / "sim.yaml"
            cfg_path.write_text(
                yaml.safe_dump({"controller": {"dt": 0.2, "sim_time": 10.0}}),
                encoding="utf-8",
            )

            args = argparse.Namespace(
                terrain="forest",
                terrain_config=None,
                controller="lqr",
                backend="pointmass",
                sim_time=1.5,
                dt=0.01,
                sim_config=str(cfg_path),
            )

            cfg = normalize_sim_config(args)
            self.assertAlmostEqual(cfg.dt, 0.01)
            self.assertAlmostEqual(cfg.sim_time, 1.5)
            self.assertEqual(cfg.controller_name, "lqr")
            self.assertEqual(cfg.backend_name, "pointmass")


if __name__ == "__main__":
    unittest.main()
