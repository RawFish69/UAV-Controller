"""Spawn Gazebo visuals/colliders from terrain MarkerArray messages.

This bridges RViz `MarkerArray` messages into Gazebo Sim by calling the `gz
service` entity creation service for simple primitive markers.
"""

from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class SpawnableMarker:
    marker_id: int
    marker_type: int
    px: float
    py: float
    pz: float
    qx: float
    qy: float
    qz: float
    qw: float
    sx: float
    sy: float
    sz: float
    cr: float
    cg: float
    cb: float
    ca: float


class TerrainMarkerSpawnerNode(Node):
    """Spawn Gazebo entities from terrain markers, once per marker set."""

    def __init__(self) -> None:
        super().__init__("terrain_marker_spawner")

        self.declare_parameter("marker_topic", "/terrain/obstacles")
        self.declare_parameter("world_name", "uav_flat_world")
        self.declare_parameter("model_name_prefix", "terrain_obs")
        self.declare_parameter("spawn_once", True)
        self.declare_parameter("create_collision", True)

        self._marker_topic = str(self.get_parameter("marker_topic").value)
        self._world_name = str(self.get_parameter("world_name").value)
        self._model_name_prefix = str(self.get_parameter("model_name_prefix").value)
        self._spawn_once = bool(self.get_parameter("spawn_once").value)
        self._create_collision = bool(self.get_parameter("create_collision").value)

        self._gz_path = shutil.which("gz")
        self._last_signature: tuple[SpawnableMarker, ...] | None = None
        self._has_spawned = False

        if self._gz_path is None:
            self.get_logger().error(
                "`gz` CLI not found on PATH. Terrain markers cannot be spawned into Gazebo."
            )
        else:
            self.get_logger().info(f"Using gz CLI at {self._gz_path}")

        self._sub = self.create_subscription(
            MarkerArray,
            self._marker_topic,
            self._on_markers,
            10,
        )
        self.get_logger().info(
            f"Listening on {self._marker_topic} and spawning into world '{self._world_name}'"
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        if self._gz_path is None:
            return

        spawnable = self._extract_spawnable(msg)
        if not spawnable:
            return

        signature = tuple(spawnable)
        if signature == self._last_signature:
            return
        self._last_signature = signature

        if self._spawn_once and self._has_spawned:
            self.get_logger().info(
                "Received changed terrain markers, but spawn_once=true so skipping respawn."
            )
            return

        self.get_logger().info(f"Spawning {len(spawnable)} terrain objects in Gazebo")
        ok_count = 0
        for idx, marker in enumerate(spawnable):
            name = f"{self._model_name_prefix}_{marker.marker_id}_{idx}"
            if self._spawn_marker(name, marker):
                ok_count += 1

        self._has_spawned = ok_count > 0
        self.get_logger().info(f"Spawned {ok_count}/{len(spawnable)} terrain objects")

    def _extract_spawnable(self, msg: MarkerArray) -> list[SpawnableMarker]:
        out: list[SpawnableMarker] = []
        unsupported = 0
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                continue
            if m.type not in (Marker.CUBE, Marker.CYLINDER, Marker.SPHERE):
                unsupported += 1
                continue
            if m.scale.x <= 0.0 or m.scale.y <= 0.0 or m.scale.z <= 0.0:
                continue
            out.append(
                SpawnableMarker(
                    marker_id=int(m.id),
                    marker_type=int(m.type),
                    px=float(m.pose.position.x),
                    py=float(m.pose.position.y),
                    pz=float(m.pose.position.z),
                    qx=float(m.pose.orientation.x),
                    qy=float(m.pose.orientation.y),
                    qz=float(m.pose.orientation.z),
                    qw=float(m.pose.orientation.w),
                    sx=float(m.scale.x),
                    sy=float(m.scale.y),
                    sz=float(m.scale.z),
                    cr=float(m.color.r),
                    cg=float(m.color.g),
                    cb=float(m.color.b),
                    ca=float(m.color.a),
                )
            )
        if unsupported:
            self.get_logger().warn(
                f"Ignored {unsupported} unsupported marker types (supports CUBE/CYLINDER/SPHERE)"
            )
        return out

    def _spawn_marker(self, model_name: str, m: SpawnableMarker) -> bool:
        sdf = self._build_sdf(model_name, m)
        service = f"/world/{self._world_name}/create"
        req = f'sdf: "{self._escape_proto_string(sdf)}"'
        cmd = [
            self._gz_path or "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"Failed to run gz service for {model_name}: {exc}")
            return False

        if proc.returncode != 0:
            stderr = (proc.stderr or "").strip()
            stdout = (proc.stdout or "").strip()
            self.get_logger().error(
                f"Spawn failed for {model_name} (rc={proc.returncode}): {stderr or stdout}"
            )
            return False
        return True

    def _build_sdf(self, model_name: str, m: SpawnableMarker) -> str:
        geometry_visual, geometry_collision = self._geometry_xml(m)
        color = f"{m.cr:.4f} {m.cg:.4f} {m.cb:.4f} {max(0.05, min(1.0, m.ca)):.4f}"
        pose = (
            f"{m.px:.4f} {m.py:.4f} {m.pz:.4f} "
            f"0 0 0"
        )
        collision_xml = (
            "<collision name='collision'>"
            f"{geometry_collision}"
            "</collision>"
            if self._create_collision
            else ""
        )
        # Terrain markers are axis-aligned today, so keep roll/pitch/yaw zero.
        # If marker orientations become meaningful later, convert quaternion to RPY.
        return "".join([
            "<?xml version='1.0'?>"
            "<sdf version='1.8'>"
            f"<model name='{model_name}'>"
            "<static>true</static>"
            f"<pose>{pose}</pose>"
            "<link name='link'>"
            f"{collision_xml}"
            "<visual name='visual'>"
            f"{geometry_visual}"
            "<material>"
            f"<ambient>{color}</ambient>"
            f"<diffuse>{color}</diffuse>"
            "<specular>0.1 0.1 0.1 1.0</specular>"
            "</material>"
            "</visual>"
            "</link>"
            "</model>"
            "</sdf>"
        ])

    def _geometry_xml(self, m: SpawnableMarker) -> tuple[str, str]:
        if m.marker_type == Marker.CUBE:
            size = f"{m.sx:.4f} {m.sy:.4f} {m.sz:.4f}"
            geom = f"<geometry><box><size>{size}</size></box></geometry>"
            return geom, geom
        if m.marker_type == Marker.SPHERE:
            radius = max(0.001, 0.5 * max(m.sx, m.sy, m.sz))
            geom = f"<geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>"
            return geom, geom
        radius = max(0.001, 0.5 * min(m.sx, m.sy))
        length = max(0.001, m.sz)
        geom = (
            "<geometry><cylinder>"
            f"<radius>{radius:.4f}</radius>"
            f"<length>{length:.4f}</length>"
            "</cylinder></geometry>"
        )
        return geom, geom

    @staticmethod
    def _escape_proto_string(text: str) -> str:
        # `gz service --req` expects a protobuf text string value.
        return text.replace("\\", "\\\\").replace('"', '\\"')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TerrainMarkerSpawnerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
