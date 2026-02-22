"""Spawn a Gazebo terrain surface mesh from the shared terrain config.

This reads the same terrain config used by planning (`sim_py.terrain_wrapper`),
extracts the `HeightFieldTerrain` (typically from the `mountains` profile), and
spawns a static mesh into Gazebo. This gives a real visible terrain surface in
Gazebo without requiring a separate hand-authored world file.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node


class TerrainSurfaceSpawnerNode(Node):
    """Generate and spawn a terrain mesh once, then exit."""

    def __init__(self) -> None:
        super().__init__("terrain_surface_spawner")

        self.declare_parameter("world_name", "uav_flat_world")
        self.declare_parameter("terrain_type", "mountains")
        self.declare_parameter("terrain_config_file", "")
        self.declare_parameter("mesh_output_dir", "/tmp/uav_controller_terrain")
        self.declare_parameter("model_name", "terrain_surface")
        self.declare_parameter("spawn_delay_sec", 2.0)

        self._world_name = str(self.get_parameter("world_name").value)
        self._terrain_type = str(self.get_parameter("terrain_type").value)
        self._terrain_config_file = str(self.get_parameter("terrain_config_file").value)
        self._mesh_output_dir = Path(str(self.get_parameter("mesh_output_dir").value))
        self._model_name = str(self.get_parameter("model_name").value)
        self._spawn_delay_sec = float(self.get_parameter("spawn_delay_sec").value)
        self._gz_path = shutil.which("gz")
        self._spawned = False

        self._timer = self.create_timer(max(0.1, self._spawn_delay_sec), self._tick)
        self.get_logger().info(
            f"Terrain surface spawner armed for world '{self._world_name}' profile '{self._terrain_type}'"
        )

    def _tick(self) -> None:
        if self._spawned:
            return
        self._timer.cancel()
        self._spawned = True

        if self._gz_path is None:
            self.get_logger().error("`gz` CLI not found on PATH")
            self._shutdown_soon()
            return

        try:
            mesh_path = self._generate_mesh()
        except Exception as exc:  # pragma: no cover - runtime integration path
            self.get_logger().error(f"Failed to generate terrain mesh: {exc}")
            self._shutdown_soon()
            return

        if mesh_path is None:
            self.get_logger().warn(
                f"No heightfield terrain generated for profile '{self._terrain_type}'. "
                "Use `mountains` for a terrain surface."
            )
            self._shutdown_soon()
            return

        if self._spawn_mesh(mesh_path):
            self.get_logger().info(f"Spawned terrain mesh: {mesh_path}")
        self._shutdown_soon()

    def _generate_mesh(self) -> Path | None:
        repo_root = self._infer_repo_root()
        if repo_root is None:
            raise RuntimeError("Could not infer repo root containing sim_py/")

        import sys

        if str(repo_root) not in sys.path:
            sys.path.append(str(repo_root))

        from sim_py.terrain_wrapper import generate_terrain, load_terrain_config, HeightFieldTerrain

        cfg = load_terrain_config(self._terrain_config_file or None)
        cfg.terrain_type = self._terrain_type
        obstacles = generate_terrain(cfg)
        heightfield = next((o for o in obstacles if isinstance(o, HeightFieldTerrain)), None)
        if heightfield is None:
            return None

        self._mesh_output_dir.mkdir(parents=True, exist_ok=True)
        mesh_path = self._mesh_output_dir / f"{self._model_name}.obj"
        self._write_obj(mesh_path, heightfield.xs, heightfield.ys, heightfield.heights)
        return mesh_path

    def _infer_repo_root(self) -> Path | None:
        here = Path(__file__).resolve()
        for parent in [here] + list(here.parents):
            if (parent / "sim_py").is_dir():
                return parent
            if (parent / "src" / "sim_gazebo").exists():
                candidate = parent.parent.parent  # .../ros2_ws/install/sim_gazebo/lib/python...
                if (candidate / "sim_py").is_dir():
                    return candidate
        cwd = Path.cwd()
        for parent in [cwd] + list(cwd.parents):
            if (parent / "sim_py").is_dir():
                return parent
        return None

    def _write_obj(self, path: Path, xs, ys, heights) -> None:
        nx = int(len(xs))
        ny = int(len(ys))
        if nx < 2 or ny < 2:
            raise RuntimeError("Heightfield grid is too small to build mesh")

        with path.open("w", encoding="utf-8") as f:
            f.write("# Generated terrain mesh\n")
            for i in range(nx):
                for j in range(ny):
                    f.write(f"v {float(xs[i]):.6f} {float(ys[j]):.6f} {float(heights[i, j]):.6f}\n")

            for i in range(nx - 1):
                for j in range(ny - 1):
                    v00 = i * ny + j + 1
                    v01 = i * ny + (j + 1) + 1
                    v10 = (i + 1) * ny + j + 1
                    v11 = (i + 1) * ny + (j + 1) + 1
                    f.write(f"f {v00} {v10} {v11}\n")
                    f.write(f"f {v00} {v11} {v01}\n")

    def _spawn_mesh(self, mesh_path: Path) -> bool:
        mesh_uri = f"file://{mesh_path}"
        sdf = (
            "<?xml version='1.0'?>"
            "<sdf version='1.8'>"
            f"<model name='{self._model_name}'>"
            "<static>true</static>"
            "<pose>0 0 0 0 0 0</pose>"
            "<link name='link'>"
            "<collision name='collision'>"
            "<geometry><mesh><uri>" + mesh_uri + "</uri></mesh></geometry>"
            "</collision>"
            "<visual name='visual'>"
            "<geometry><mesh><uri>" + mesh_uri + "</uri></mesh></geometry>"
            "<material>"
            "<ambient>0.45 0.42 0.38 1</ambient>"
            "<diffuse>0.55 0.52 0.48 1</diffuse>"
            "<specular>0.08 0.08 0.08 1</specular>"
            "</material>"
            "</visual>"
            "</link>"
            "</model>"
            "</sdf>"
        )

        cmd = [
            self._gz_path or "gz",
            "service",
            "-s",
            f"/world/{self._world_name}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f'sdf: "{self._escape_proto_string(sdf)}"',
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if proc.returncode != 0:
            self.get_logger().error(
                f"Gazebo terrain mesh spawn failed (rc={proc.returncode}): "
                f"{(proc.stderr or proc.stdout).strip()}"
            )
            return False
        return True

    def _shutdown_soon(self) -> None:
        self.create_timer(0.2, lambda: rclpy.shutdown())

    @staticmethod
    def _escape_proto_string(text: str) -> str:
        return text.replace("\\", "\\\\").replace('"', '\\"')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TerrainSurfaceSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
