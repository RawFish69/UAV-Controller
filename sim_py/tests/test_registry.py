from __future__ import annotations

import unittest

from sim_py.core.registry import (
    create_backend,
    create_controller,
    create_planner,
    register_builtin_components,
)


class TestRegistry(unittest.TestCase):
    def setUp(self) -> None:
        register_builtin_components()

    def test_builtin_components_resolve(self) -> None:
        create_planner("straight")
        create_planner("rrt*")
        create_controller("pid")
        create_backend("pointmass")

    def test_unknown_component_raises_helpful_error(self) -> None:
        with self.assertRaises(ValueError) as e1:
            create_planner("does-not-exist")
        self.assertIn("Unknown planner", str(e1.exception))

        with self.assertRaises(ValueError) as e2:
            create_controller("does-not-exist")
        self.assertIn("Unknown controller", str(e2.exception))

        with self.assertRaises(ValueError) as e3:
            create_backend("does-not-exist")
        self.assertIn("Unknown backend", str(e3.exception))


if __name__ == "__main__":
    unittest.main()
