"""Mirror planner MarkerArray path visuals into Gazebo Sim.

Supports:
- Marker.SPHERE (waypoint dots)
- Marker.LINE_STRIP (rendered as cylinder segments)
- Marker.DELETEALL (remove all tracked path entities)
"""

from __future__ import annotations

import math
import shutil
import subprocess
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class PrimitiveVisual:
    kind: str  # "sphere" | "cylinder"
    key: str
    px: float
    py: float
    pz: float
    roll: float
    pitch: float
    yaw: float
    sx: float
    sy: float
    sz: float
    cr: float
    cg: float
    cb: float
    ca: float


class PathMarkerSpawnerNode(Node):
    """Spawn/update Gazebo entities from planner path MarkerArray messages."""

    def __init__(self) -> None:
        super().__init__('path_marker_spawner')

        self.declare_parameter('marker_topic', '/gs/planner/planned_path_markers')
        self.declare_parameter('world_name', 'uav_flat_world')
        self.declare_parameter('model_name_prefix', 'planned_path')
        self.declare_parameter('spawn_once', False)
        self.declare_parameter('create_collision', False)
        self.declare_parameter('skip_glow_namespaces', True)
        self.declare_parameter('line_diameter_scale', 0.8)
        self.declare_parameter('line_diameter_min_m', 0.2)
        self.declare_parameter('line_diameter_max_m', 1.2)
        self.declare_parameter('sphere_diameter_scale', 1.0)
        self.declare_parameter('sphere_diameter_min_m', 0.5)
        self.declare_parameter('sphere_diameter_max_m', 2.5)
        self.declare_parameter('z_offset_m', 0.05)

        self._marker_topic = str(self.get_parameter('marker_topic').value)
        self._world_name = str(self.get_parameter('world_name').value)
        self._model_name_prefix = str(self.get_parameter('model_name_prefix').value)
        self._spawn_once = bool(self.get_parameter('spawn_once').value)
        self._create_collision = bool(self.get_parameter('create_collision').value)
        self._skip_glow_namespaces = bool(self.get_parameter('skip_glow_namespaces').value)
        self._line_diameter_scale = float(self.get_parameter('line_diameter_scale').value)
        self._line_diameter_min_m = float(self.get_parameter('line_diameter_min_m').value)
        self._line_diameter_max_m = float(self.get_parameter('line_diameter_max_m').value)
        self._sphere_diameter_scale = float(self.get_parameter('sphere_diameter_scale').value)
        self._sphere_diameter_min_m = float(self.get_parameter('sphere_diameter_min_m').value)
        self._sphere_diameter_max_m = float(self.get_parameter('sphere_diameter_max_m').value)
        self._z_offset_m = float(self.get_parameter('z_offset_m').value)

        self._gz_path = shutil.which('gz')
        self._last_signature: tuple[PrimitiveVisual, ...] | None = None
        self._tracked_entities: set[str] = set()
        self._has_spawned = False

        if self._gz_path is None:
            self.get_logger().error('`gz` CLI not found on PATH. Path visuals cannot be spawned.')
        else:
            self.get_logger().info(f'Using gz CLI at {self._gz_path}')

        # TRANSIENT_LOCAL matches the planner publisher so we receive the last retained path
        # even when this node starts after planning has already finished.
        _durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub = self.create_subscription(MarkerArray, self._marker_topic, self._on_markers, _durable_qos)
        self.get_logger().info(
            f"Listening on {self._marker_topic} and mirroring path visuals into world '{self._world_name}'"
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        if self._gz_path is None:
            return

        has_delete_all = any(int(m.action) == Marker.DELETEALL for m in msg.markers)
        primitives = self._extract_primitives(msg)
        signature = tuple(primitives)
        if len(primitives) > 0:
            if signature != self._last_signature:
                self.get_logger().info(
                    f'Path markers received: {len(msg.markers)} markers -> {len(primitives)} primitives (path finder)'
                )

        if has_delete_all and self._tracked_entities:
            self._clear_tracked_entities()
            self._has_spawned = False
            self._last_signature = None

        if not primitives:
            return

        if signature == self._last_signature:
            return
        if self._spawn_once and self._has_spawned:
            self.get_logger().info(
                'Received changed path markers, but spawn_once=true so skipping respawn.'
            )
            return

        if self._tracked_entities:
            self._clear_tracked_entities()

        ok_count = 0
        new_entities: set[str] = set()
        for idx, p in enumerate(primitives):
            model_name = f'{self._model_name_prefix}_{p.key}_{idx}'
            if self._spawn_primitive(model_name, p):
                ok_count += 1
                new_entities.add(model_name)

        self._tracked_entities = new_entities
        self._last_signature = signature
        self._has_spawned = ok_count > 0
        self.get_logger().info(f'Spawned {ok_count}/{len(primitives)} path visuals into Gazebo')
        if ok_count == 0 and len(primitives) > 0:
            self.get_logger().warn(
                'Path visuals: 0 spawned (Gazebo create may have failed; check world is running and world_name=uav_flat_world)'
            )

    def _extract_primitives(self, msg: MarkerArray) -> list[PrimitiveVisual]:
        out: list[PrimitiveVisual] = []
        unsupported = 0
        for m in msg.markers:
            if int(m.action) == Marker.DELETEALL:
                continue
            if int(m.action) != Marker.ADD:
                continue
            if self._skip_glow_namespaces and 'glow' in str(m.ns).lower():
                continue
            if int(m.type) == Marker.SPHERE:
                if m.scale.x <= 0.0 or m.scale.y <= 0.0 or m.scale.z <= 0.0:
                    continue
                sphere_diameter = self._scaled_sphere_diameter(
                    max(float(m.scale.x), float(m.scale.y), float(m.scale.z))
                )
                out.append(
                    PrimitiveVisual(
                        kind='sphere',
                        key=f'{m.ns}_{int(m.id)}',
                        px=float(m.pose.position.x),
                        py=float(m.pose.position.y),
                        pz=float(m.pose.position.z) + self._z_offset_m,
                        roll=0.0,
                        pitch=0.0,
                        yaw=0.0,
                        sx=sphere_diameter,
                        sy=sphere_diameter,
                        sz=sphere_diameter,
                        cr=float(m.color.r),
                        cg=float(m.color.g),
                        cb=float(m.color.b),
                        ca=float(m.color.a),
                    )
                )
                continue
            if int(m.type) == Marker.LINE_STRIP:
                if len(m.points) < 2 or m.scale.x <= 0.0:
                    continue
                line_diameter = self._scaled_line_diameter(float(m.scale.x))
                radius = max(0.001, 0.5 * line_diameter)
                for seg_idx in range(len(m.points) - 1):
                    p0 = m.points[seg_idx]
                    p1 = m.points[seg_idx + 1]
                    dx = float(p1.x - p0.x)
                    dy = float(p1.y - p0.y)
                    dz = float(p1.z - p0.z)
                    seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if seg_len <= 1e-6:
                        continue
                    roll, pitch, yaw = self._rpy_from_z_axis(dx / seg_len, dy / seg_len, dz / seg_len)
                    out.append(
                        PrimitiveVisual(
                            kind='cylinder',
                            key=f'{m.ns}_{int(m.id)}_seg{seg_idx}',
                            px=float((p0.x + p1.x) * 0.5),
                            py=float((p0.y + p1.y) * 0.5),
                            pz=float((p0.z + p1.z) * 0.5) + self._z_offset_m,
                            roll=roll,
                            pitch=pitch,
                            yaw=yaw,
                            sx=2.0 * radius,
                            sy=2.0 * radius,
                            sz=seg_len,
                            cr=float(m.color.r),
                            cg=float(m.color.g),
                            cb=float(m.color.b),
                            ca=float(m.color.a),
                        )
                    )
                continue
            unsupported += 1
        if unsupported:
            self.get_logger().warn(
                f'Ignored {unsupported} unsupported path marker types (supports SPHERE/LINE_STRIP)'
            )
        return out

    def _scaled_line_diameter(self, rviz_width_m: float) -> float:
        raw = float(rviz_width_m) * self._line_diameter_scale
        return self._clamp(raw, self._line_diameter_min_m, self._line_diameter_max_m)

    def _scaled_sphere_diameter(self, rviz_diameter_m: float) -> float:
        raw = float(rviz_diameter_m) * self._sphere_diameter_scale
        return self._clamp(raw, self._sphere_diameter_min_m, self._sphere_diameter_max_m)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        lo = float(low)
        hi = float(high)
        if hi < lo:
            lo, hi = hi, lo
        return max(lo, min(hi, float(value)))

    def _clear_tracked_entities(self) -> None:
        names = sorted(self._tracked_entities)
        removed = 0
        for name in names:
            if self._remove_entity(name):
                removed += 1
        if names:
            self.get_logger().info(f'Removed {removed}/{len(names)} existing path visuals')
        self._tracked_entities.clear()

    def _spawn_primitive(self, model_name: str, p: PrimitiveVisual) -> bool:
        sdf = self._build_sdf(model_name, p)
        return self._call_gz_service(
            service=f'/world/{self._world_name}/create',
            reqtype='gz.msgs.EntityFactory',
            reptype='gz.msgs.Boolean',
            req=f'sdf: "{self._escape_proto_string(sdf)}"',
        )

    def _remove_entity(self, model_name: str) -> bool:
        # Gazebo versions vary in enum parsing; try symbolic then numeric MODEL enum.
        requests = [
            f'name: "{model_name}" type: MODEL',
            f'name: "{model_name}" type: 2',
            f'name: "{model_name}"',
        ]
        for req in requests:
            if self._call_gz_service(
                service=f'/world/{self._world_name}/remove',
                reqtype='gz.msgs.Entity',
                reptype='gz.msgs.Boolean',
                req=req,
                suppress_logs=True,
            ):
                return True
        self.get_logger().warn(f'Failed to remove Gazebo entity {model_name}')
        return False

    def _call_gz_service(
        self,
        service: str,
        reqtype: str,
        reptype: str,
        req: str,
        suppress_logs: bool = False,
    ) -> bool:
        cmd = [
            self._gz_path or 'gz',
            'service',
            '-s',
            service,
            '--reqtype',
            reqtype,
            '--reptype',
            reptype,
            '--timeout',
            '5000',
            '--req',
            req,
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        except Exception as exc:  # pragma: no cover
            if not suppress_logs:
                self.get_logger().error(f'Failed to run gz service {service}: {exc}')
            return False
        if proc.returncode != 0:
            if not suppress_logs:
                msg = (proc.stderr or proc.stdout or '').strip()
                self.get_logger().error(
                    f'Gazebo service call failed {service} (rc={proc.returncode}): {msg}'
                )
            return False
        stdout = (proc.stdout or '').lower()
        if 'data: true' in stdout or 'success: true' in stdout:
            return True
        # Some versions emit empty stdout despite success; treat rc==0 as success.
        return True

    def _build_sdf(self, model_name: str, p: PrimitiveVisual) -> str:
        geometry_visual, geometry_collision = self._geometry_xml(p)
        color = f'{p.cr:.4f} {p.cg:.4f} {p.cb:.4f} {max(0.05, min(1.0, p.ca)):.4f}'
        pose = f'{p.px:.4f} {p.py:.4f} {p.pz:.4f} {p.roll:.6f} {p.pitch:.6f} {p.yaw:.6f}'
        collision_xml = (
            "<collision name='collision'>"
            f'{geometry_collision}'
            '</collision>'
            if self._create_collision
            else ''
        )
        return ''.join(
            [
                "<?xml version='1.0'?>",
                "<sdf version='1.8'>",
                f"<model name='{model_name}'>",
                '<static>true</static>',
                f'<pose>{pose}</pose>',
                "<link name='link'>",
                collision_xml,
                "<visual name='visual'>",
                geometry_visual,
                '<material>',
                f'<ambient>{color}</ambient>',
                f'<diffuse>{color}</diffuse>',
                '<specular>0.1 0.1 0.1 1.0</specular>',
                '</material>',
                '</visual>',
                '</link>',
                '</model>',
                '</sdf>',
            ]
        )

    def _geometry_xml(self, p: PrimitiveVisual) -> tuple[str, str]:
        if p.kind == 'sphere':
            radius = max(0.001, 0.5 * max(p.sx, p.sy, p.sz))
            geom = f'<geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>'
            return geom, geom
        radius = max(0.001, 0.5 * min(p.sx, p.sy))
        length = max(0.001, p.sz)
        geom = (
            '<geometry><cylinder>'
            f'<radius>{radius:.4f}</radius>'
            f'<length>{length:.4f}</length>'
            '</cylinder></geometry>'
        )
        return geom, geom

    @staticmethod
    def _rpy_from_z_axis(ux: float, uy: float, uz: float) -> tuple[float, float, float]:
        # Quaternion rotating +Z to the target unit vector, then convert to RPY.
        z_dot = max(-1.0, min(1.0, uz))
        if z_dot < -0.999999:
            # 180 deg about X axis maps +Z to -Z.
            qx, qy, qz, qw = 1.0, 0.0, 0.0, 0.0
        else:
            cx = -uy
            cy = ux
            cz = 0.0
            s = math.sqrt((1.0 + z_dot) * 2.0)
            if s <= 1e-9:
                qx = qy = qz = 0.0
                qw = 1.0
            else:
                inv_s = 1.0 / s
                qx = cx * inv_s
                qy = cy * inv_s
                qz = cz * inv_s
                qw = 0.5 * s
        return PathMarkerSpawnerNode._quat_to_rpy(qx, qy, qz, qw)

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _escape_proto_string(text: str) -> str:
        return text.replace('\\', '\\\\').replace('"', '\\"')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PathMarkerSpawnerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
