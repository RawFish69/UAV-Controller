#!/usr/bin/env python3
"""Generate/patch lr_drone model alignment values into URDF/SDF files.

This keeps rotor joint origins, prop mesh offsets / orientations, and the simplified frame
collision primitive in sync across:
  - models/lr_drone_urdf/lr_drone_urdf.urdf      (RViz preview)
  - models/lr_drone_urdf/lr_drone_gazebo.urdf    (Gazebo preview)
  - models/lr_drone_urdf/model.sdf               (Gazebo preview, static SDF)
  - models/lr_drone_controlled/model.sdf         (Gazebo controlled sim)

The alignment constants live in:
  models/lr_drone_urdf/lr_drone_alignment.yaml

Notes:
  - CW prop meshes use a different CAD-local mesh frame than CCW meshes, so
    their larger X mesh offsets are intentional and should be preserved.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception as exc:  # pragma: no cover - runtime tool dependency
    raise SystemExit(
        "PyYAML is required to run generate_lr_drone_models.py "
        f"(import failed: {exc})"
    )


def _fmt_float(value: float) -> str:
    text = f"{float(value):.16g}"
    if text == "-0":
        return "0"
    return text


def _fmt_xyz(vals: list[float] | tuple[float, float, float]) -> str:
    return " ".join(_fmt_float(v) for v in vals)


def _fmt_rpy(vals: list[float] | tuple[float, float, float]) -> str:
    return " ".join(_fmt_float(v) for v in vals)


def _fmt_pose6(
    xyz: list[float] | tuple[float, float, float],
    rpy: list[float] | tuple[float, float, float] | None = None,
) -> str:
    rpy_vals = (0.0, 0.0, 0.0) if rpy is None else rpy
    return f"{_fmt_xyz(xyz)} {_fmt_rpy(rpy_vals)}"


def _vec_add(a: list[float], b: list[float]) -> list[float]:
    return [float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2])]


def _as_vec3(value: Any, *, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field} must be a 3-element list/tuple, got: {value!r}")
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except Exception as exc:
        raise ValueError(f"{field} must contain numeric values: {value!r}") from exc


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Alignment YAML must be a mapping: {path}")
    return data


def _sub_one(text: str, pattern: re.Pattern[str], repl, desc: str, *, optional: bool = False) -> str:
    new_text, count = pattern.subn(repl, text, count=1)
    if count == 0 and not optional:
        raise ValueError(f"Pattern not found for {desc}")
    return new_text


def _replace_urdf_joint_origin(text: str, joint_name: str, motor_xyz: list[float]) -> str:
    pat = re.compile(
        rf'(<joint name="{re.escape(joint_name)}"[^>]*>.*?<origin xyz=")([^"]+)(" rpy=")([^"]+)("\s*/>)',
        re.S,
    )
    return _sub_one(
        text,
        pat,
        lambda m: f'{m.group(1)}{_fmt_xyz(motor_xyz)}{m.group(3)}{m.group(4)}{m.group(5)}',
        f"URDF joint origin {joint_name}",
    )


def _replace_urdf_link_origin(
    text: str,
    *,
    link_name: str,
    tag_name: str,
    origin_xyz: list[float],
    origin_rpy: list[float] | None = None,
    optional: bool = False,
) -> str:
    pat = re.compile(
        rf'(<link name="{re.escape(link_name)}"[^>]*>.*?<{tag_name} name="[^"]*">.*?<origin xyz=")([^"]+)(" rpy=")([^"]+)("\s*/>)',
        re.S,
    )
    rpy_vals = [0.0, 0.0, 0.0] if origin_rpy is None else origin_rpy
    return _sub_one(
        text,
        pat,
        lambda m: f'{m.group(1)}{_fmt_xyz(origin_xyz)}{m.group(3)}{_fmt_rpy(rpy_vals)}{m.group(5)}',
        f"URDF {tag_name} origin {link_name}",
        optional=optional,
    )


def _replace_gazebo_urdf_frame_collision(text: str, center_xyz: list[float], size_xyz: list[float]) -> str:
    origin_pat = re.compile(
        r'(<collision name="frame_collision">.*?<origin xyz=")([^"]+)(" rpy="[^"]*"\s*/>)',
        re.S,
    )
    size_pat = re.compile(
        r'(<collision name="frame_collision">.*?<box size=")([^"]+)("\s*/>)',
        re.S,
    )
    text = _sub_one(
        text,
        origin_pat,
        lambda m: f'{m.group(1)}{_fmt_xyz(center_xyz)}{m.group(3)}',
        "Gazebo URDF frame collision origin",
    )
    text = _sub_one(
        text,
        size_pat,
        lambda m: f'{m.group(1)}{_fmt_xyz(size_xyz)}{m.group(3)}',
        "Gazebo URDF frame collision size",
    )
    return text


def _replace_sdf_joint_pose(text: str, joint_name: str, motor_xyz: list[float]) -> str:
    pat = re.compile(
        rf'(<joint name="{re.escape(joint_name)}"[^>]*>.*?<pose relative_to="frame">)([^<]+)(</pose>)',
        re.S,
    )
    return _sub_one(
        text,
        pat,
        lambda m: f'{m.group(1)}{_fmt_pose6(motor_xyz)}{m.group(3)}',
        f"SDF joint pose {joint_name}",
    )


def _replace_sdf_prop_visual_pose(
    text: str,
    link_name: str,
    origin_xyz: list[float],
    origin_rpy: list[float] | None = None,
) -> str:
    pat = re.compile(
        rf'(<link name="{re.escape(link_name)}"[^>]*>.*?<visual name="[^"]*">.*?<pose>)([^<]+)(</pose>)',
        re.S,
    )
    rpy_vals = [0.0, 0.0, 0.0] if origin_rpy is None else origin_rpy
    return _sub_one(
        text,
        pat,
        lambda m: f'{m.group(1)}{_fmt_pose6(origin_xyz, rpy_vals)}{m.group(3)}',
        f"SDF visual pose {link_name}",
    )


def _build_controlled_rotor_visual_block(
    link_name: str,
    origin_xyz: list[float],
    origin_rpy: list[float] | None = None,
) -> str:
    rpy_vals = [0.0, 0.0, 0.0] if origin_rpy is None else origin_rpy
    lines: list[str] = []
    if "Prop_CW" in link_name:
        lines.append(
            "      <!-- CW props were exported in a different CAD-local mesh frame; the large X offset is intentional. -->"
        )
    lines.extend([
        f'      <visual name="{link_name}_visual_visual">',
        f'        <pose>{_fmt_pose6(origin_xyz, rpy_vals)}</pose>',
        '        <geometry>',
        '          <mesh>',
        '            <scale>0.001 0.001 0.001</scale>',
        f'            <uri>model://lr_drone_urdf/meshes/{link_name}.stl</uri>',
        '          </mesh>',
        '        </geometry>',
        '      </visual>',
    ])
    return "\n".join(lines)


def _upsert_sdf_link_visual_block(text: str, link_name: str, visual_block: str) -> str:
    replace_pat = re.compile(
        rf'(<link name="{re.escape(link_name)}"[^>]*>.*?)(\n\s*<visual name="[^"]*">.*?</visual>)(.*?</link>)',
        re.S,
    )
    new_text, count = replace_pat.subn(lambda m: f"{m.group(1)}\n{visual_block}{m.group(3)}", text, count=1)
    if count == 1:
        return new_text

    insert_pat = re.compile(
        rf'(<link name="{re.escape(link_name)}"[^>]*>.*?)(\n\s*</link>)',
        re.S,
    )
    return _sub_one(
        text,
        insert_pat,
        lambda m: f"{m.group(1)}\n{visual_block}{m.group(2)}",
        f"SDF insert visual block {link_name}",
    )


def _remove_sdf_link_visual_block(text: str, link_name: str) -> str:
    pat = re.compile(
        rf'(<link name="{re.escape(link_name)}"[^>]*>.*?)(\n\s*<visual name="[^"]*">.*?</visual>)(.*?</link>)',
        re.S,
    )
    new_text, count = pat.subn(lambda m: f"{m.group(1)}{m.group(3)}", text, count=1)
    if count == 0:
        return text
    return new_text


def _upsert_sdf_prop_visual_pose(
    text: str,
    link_name: str,
    origin_xyz: list[float],
    origin_rpy: list[float] | None = None,
) -> str:
    try:
        return _replace_sdf_prop_visual_pose(text, link_name, origin_xyz, origin_rpy)
    except ValueError:
        block = _build_controlled_rotor_visual_block(link_name, origin_xyz, origin_rpy)
        return _upsert_sdf_link_visual_block(text, link_name, block)


def _replace_sdf_frame_collision(text: str, center_xyz: list[float], size_xyz: list[float]) -> str:
    pose_pat = re.compile(
        r'(<collision name="frame_collision_collision">.*?<pose>)([^<]+)(</pose>)',
        re.S,
    )
    size_pat = re.compile(
        r'(<collision name="frame_collision_collision">.*?<size>)([^<]+)(</size>)',
        re.S,
    )
    text = _sub_one(
        text,
        pose_pat,
        lambda m: f'{m.group(1)}{_fmt_pose6(center_xyz)}{m.group(3)}',
        "SDF frame collision pose",
    )
    text = _sub_one(
        text,
        size_pat,
        lambda m: f'{m.group(1)}{_fmt_xyz(size_xyz)}{m.group(3)}',
        "SDF frame collision size",
    )
    return text


def _controlled_prop_visual_mode(alignment: dict[str, Any]) -> str:
    cfg = alignment.get("controlled_sim", {})
    if cfg is None:
        cfg = {}
    if not isinstance(cfg, dict):
        raise ValueError(f"controlled_sim must be a mapping if provided, got: {cfg!r}")
    mode = str(cfg.get("prop_visual_mode", "rotor_link")).strip().lower()
    if mode not in {"rotor_link", "frame_fixed"}:
        raise ValueError(
            f"controlled_sim.prop_visual_mode must be 'rotor_link' or 'frame_fixed', got: {mode!r}"
        )
    return mode


def _build_controlled_frame_fixed_prop_visuals_block(alignment: dict[str, Any]) -> str:
    rotors: dict[str, Any] = dict(alignment["rotors"])
    lines: list[str] = [
        "      <!-- BEGIN GENERATED_CONTROLLED_FIXED_PROP_VISUALS -->",
        "      <!-- Frame-fixed CAD prop visuals for stable-flight-first Gazebo rendering (rotor links remain for plugins). -->",
    ]
    for joint_name, rotor in rotors.items():
        child_link = str(rotor["child_link"])
        motor_xyz = _as_vec3(rotor["motor_xyz_m"], field=f"rotors.{joint_name}.motor_xyz_m")
        mesh_origin = _as_vec3(rotor["mesh_origin_xyz_m"], field=f"rotors.{joint_name}.mesh_origin_xyz_m")
        mesh_rpy = _as_vec3(
            rotor.get("mesh_origin_rpy_rad", [0.0, 0.0, 0.0]),
            field=f"rotors.{joint_name}.mesh_origin_rpy_rad",
        )
        pose_xyz = _vec_add(motor_xyz, mesh_origin)
        if "Prop_CW" in child_link:
            lines.append(
                "      <!-- CW props were exported in a different CAD-local mesh frame; the large X offset is intentional. -->"
            )
        lines.extend([
            f'      <visual name="{child_link}_frame_fixed_visual">',
            f'        <pose>{_fmt_pose6(pose_xyz, mesh_rpy)}</pose>',
            '        <geometry>',
            '          <mesh>',
            '            <scale>0.001 0.001 0.001</scale>',
            f'            <uri>model://lr_drone_urdf/meshes/{child_link}.stl</uri>',
            '          </mesh>',
            '        </geometry>',
            '      </visual>',
        ])
    lines.append("      <!-- END GENERATED_CONTROLLED_FIXED_PROP_VISUALS -->")
    return "\n".join(lines)


def _remove_controlled_frame_fixed_prop_visuals_block(text: str) -> str:
    pat = re.compile(
        r"\n\s*<!-- BEGIN GENERATED_CONTROLLED_FIXED_PROP_VISUALS -->.*?<!-- END GENERATED_CONTROLLED_FIXED_PROP_VISUALS -->",
        re.S,
    )
    return pat.sub("", text, count=1)


def _set_controlled_frame_fixed_prop_visuals_block(text: str, alignment: dict[str, Any]) -> str:
    text = _remove_controlled_frame_fixed_prop_visuals_block(text)
    block = _build_controlled_frame_fixed_prop_visuals_block(alignment)
    frame_pat = re.compile(r'(<link name="frame">.*?)(\n\s*</link>)', re.S)
    return _sub_one(
        text,
        frame_pat,
        lambda m: f"{m.group(1)}\n{block}{m.group(2)}",
        "controlled frame fixed prop visuals block",
    )


def _apply_to_rviz_urdf(text: str, alignment: dict[str, Any]) -> str:
    rotors: dict[str, Any] = dict(alignment["rotors"])
    for joint_name, rotor in rotors.items():
        motor_xyz = _as_vec3(rotor["motor_xyz_m"], field=f"rotors.{joint_name}.motor_xyz_m")
        mesh_origin = _as_vec3(rotor["mesh_origin_xyz_m"], field=f"rotors.{joint_name}.mesh_origin_xyz_m")
        mesh_rpy = _as_vec3(
            rotor.get("mesh_origin_rpy_rad", [0.0, 0.0, 0.0]),
            field=f"rotors.{joint_name}.mesh_origin_rpy_rad",
        )
        text = _replace_urdf_joint_origin(text, joint_name, motor_xyz)
        link_name = str(rotor["child_link"])
        text = _replace_urdf_link_origin(
            text, link_name=link_name, tag_name="visual", origin_xyz=mesh_origin, origin_rpy=mesh_rpy
        )
        text = _replace_urdf_link_origin(
            text,
            link_name=link_name,
            tag_name="collision",
            origin_xyz=mesh_origin,
            origin_rpy=mesh_rpy,
            optional=False,
        )
    return text


def _apply_to_gazebo_urdf(text: str, alignment: dict[str, Any]) -> str:
    frame = dict(alignment["frame_collision"])
    text = _replace_gazebo_urdf_frame_collision(
        text,
        _as_vec3(frame["center_xyz_m"], field="frame_collision.center_xyz_m"),
        _as_vec3(frame["box_size_xyz_m"], field="frame_collision.box_size_xyz_m"),
    )
    rotors: dict[str, Any] = dict(alignment["rotors"])
    for joint_name, rotor in rotors.items():
        motor_xyz = _as_vec3(rotor["motor_xyz_m"], field=f"rotors.{joint_name}.motor_xyz_m")
        mesh_origin = _as_vec3(rotor["mesh_origin_xyz_m"], field=f"rotors.{joint_name}.mesh_origin_xyz_m")
        mesh_rpy = _as_vec3(
            rotor.get("mesh_origin_rpy_rad", [0.0, 0.0, 0.0]),
            field=f"rotors.{joint_name}.mesh_origin_rpy_rad",
        )
        text = _replace_urdf_joint_origin(text, joint_name, motor_xyz)
        link_name = str(rotor["child_link"])
        text = _replace_urdf_link_origin(
            text, link_name=link_name, tag_name="visual", origin_xyz=mesh_origin, origin_rpy=mesh_rpy
        )
        # Gazebo preview URDF omits prop collisions intentionally.
        text = _replace_urdf_link_origin(
            text,
            link_name=link_name,
            tag_name="collision",
            origin_xyz=mesh_origin,
            origin_rpy=mesh_rpy,
            optional=True,
        )
    return text


def _apply_to_controlled_sdf(text: str, alignment: dict[str, Any]) -> str:
    frame = dict(alignment["frame_collision"])
    text = _replace_sdf_frame_collision(
        text,
        _as_vec3(frame["center_xyz_m"], field="frame_collision.center_xyz_m"),
        _as_vec3(frame["box_size_xyz_m"], field="frame_collision.box_size_xyz_m"),
    )
    mode = _controlled_prop_visual_mode(alignment)
    rotors: dict[str, Any] = dict(alignment["rotors"])
    for joint_name, rotor in rotors.items():
        motor_xyz = _as_vec3(rotor["motor_xyz_m"], field=f"rotors.{joint_name}.motor_xyz_m")
        mesh_origin = _as_vec3(rotor["mesh_origin_xyz_m"], field=f"rotors.{joint_name}.mesh_origin_xyz_m")
        mesh_rpy = _as_vec3(
            rotor.get("mesh_origin_rpy_rad", [0.0, 0.0, 0.0]),
            field=f"rotors.{joint_name}.mesh_origin_rpy_rad",
        )
        text = _replace_sdf_joint_pose(text, joint_name, motor_xyz)
        child_link = str(rotor["child_link"])
        if mode == "rotor_link":
            text = _upsert_sdf_prop_visual_pose(text, child_link, mesh_origin, mesh_rpy)
        else:
            text = _remove_sdf_link_visual_block(text, child_link)
    if mode == "frame_fixed":
        text = _set_controlled_frame_fixed_prop_visuals_block(text, alignment)
    else:
        text = _remove_controlled_frame_fixed_prop_visuals_block(text)
    return text


def _render_preview_sdf(alignment: dict[str, Any]) -> str:
    rotors: dict[str, Any] = dict(alignment["rotors"])
    lines: list[str] = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.8">',
        '  <model name="lr_drone_urdf">',
        '    <!--',
        '      Static visual-only Gazebo preview model generated from lr_drone_alignment.yaml.',
        '      RViz continues to use lr_drone_urdf.urdf directly; Gazebo preview uses SDF here',
        '      to avoid URDF->SDF conversion differences (fixed-joint reduction / physics jitter).',
        '      The preview SDF keeps the same frame->prop link structure as the URDF for transform parity.',
        '    -->',
        '    <static>true</static>',
        '    <self_collide>false</self_collide>',
        '    <allow_auto_disable>true</allow_auto_disable>',
        '    <link name="frame">',
        '      <pose>0 0 0 0 0 0</pose>',
        '      <visual name="frame_visual">',
        '        <pose>0 0 0 0 0 0</pose>',
        '        <geometry>',
        '          <mesh>',
        '            <uri>model://lr_drone_urdf/meshes/frame.stl</uri>',
        '            <scale>0.001 0.001 0.001</scale>',
        '          </mesh>',
        '        </geometry>',
        '      </visual>',
        '    </link>',
    ]

    for joint_name, rotor in rotors.items():
        child_link = str(rotor["child_link"])
        motor_xyz = _as_vec3(rotor["motor_xyz_m"], field=f"rotors.{joint_name}.motor_xyz_m")
        mesh_origin = _as_vec3(rotor["mesh_origin_xyz_m"], field=f"rotors.{joint_name}.mesh_origin_xyz_m")
        mesh_rpy = _as_vec3(
            rotor.get("mesh_origin_rpy_rad", [0.0, 0.0, 0.0]),
            field=f"rotors.{joint_name}.mesh_origin_rpy_rad",
        )
        mesh_file = f"{child_link}.stl"
        if "Prop_CW" in child_link:
            lines.append(
                "    <!-- CW props were exported in a different CAD-local mesh frame; large X offsets are intentional. -->"
            )
        lines.extend([
            f'    <link name="{child_link}">',
            f'      <pose>{_fmt_pose6(motor_xyz)}</pose>',
            f'      <visual name="{child_link}_visual">',
            f'        <pose>{_fmt_pose6(mesh_origin, mesh_rpy)}</pose>',
            '        <geometry>',
            '          <mesh>',
            f'            <uri>model://lr_drone_urdf/meshes/{mesh_file}</uri>',
            '            <scale>0.001 0.001 0.001</scale>',
            '          </mesh>',
            '        </geometry>',
            '      </visual>',
            '    </link>',
            f'    <joint name="{joint_name}" type="fixed">',
            '      <parent>frame</parent>',
            f'      <child>{child_link}</child>',
            '      <pose>0 0 0 0 0 0</pose>',
            '    </joint>',
        ])

    lines.extend([
        '  </model>',
        '</sdf>',
        '',
    ])
    return "\n".join(lines)


def _default_paths(script_path: Path) -> tuple[Path, Path, Path, Path, Path]:
    sim_gazebo_dir = script_path.resolve().parents[1]
    models_dir = sim_gazebo_dir / "models"
    alignment = models_dir / "lr_drone_urdf" / "lr_drone_alignment.yaml"
    rviz_urdf = models_dir / "lr_drone_urdf" / "lr_drone_urdf.urdf"
    gazebo_urdf = models_dir / "lr_drone_urdf" / "lr_drone_gazebo.urdf"
    preview_sdf = models_dir / "lr_drone_urdf" / "model.sdf"
    controlled_sdf = models_dir / "lr_drone_controlled" / "model.sdf"
    return alignment, rviz_urdf, gazebo_urdf, preview_sdf, controlled_sdf


def _process_file(path: Path, new_text: str, *, check: bool) -> bool:
    old_text = path.read_text(encoding="utf-8") if path.exists() else None
    changed = old_text != new_text
    if changed and not check:
        path.write_text(new_text, encoding="utf-8")
    return changed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check whether generated content matches files (no writes, exits non-zero on diff).",
    )
    parser.add_argument(
        "--alignment",
        type=Path,
        default=None,
        help="Path to lr_drone_alignment.yaml (defaults to repo sim_gazebo models path).",
    )
    args = parser.parse_args(argv)

    (
        alignment_path_default,
        rviz_urdf_path,
        gazebo_urdf_path,
        preview_sdf_path,
        controlled_sdf_path,
    ) = _default_paths(
        Path(__file__)
    )
    alignment_path = args.alignment or alignment_path_default

    alignment = _load_yaml(alignment_path)
    for key in ("rotors", "frame_collision"):
        if key not in alignment:
            raise ValueError(f"Missing required key '{key}' in {alignment_path}")

    rviz_urdf_text = rviz_urdf_path.read_text(encoding="utf-8")
    gazebo_urdf_text = gazebo_urdf_path.read_text(encoding="utf-8")
    controlled_sdf_text = controlled_sdf_path.read_text(encoding="utf-8")

    rviz_urdf_new = _apply_to_rviz_urdf(rviz_urdf_text, alignment)
    gazebo_urdf_new = _apply_to_gazebo_urdf(gazebo_urdf_text, alignment)
    preview_sdf_new = _render_preview_sdf(alignment)
    controlled_sdf_new = _apply_to_controlled_sdf(controlled_sdf_text, alignment)

    changed: list[Path] = []
    if _process_file(rviz_urdf_path, rviz_urdf_new, check=args.check):
        changed.append(rviz_urdf_path)
    if _process_file(gazebo_urdf_path, gazebo_urdf_new, check=args.check):
        changed.append(gazebo_urdf_path)
    if _process_file(preview_sdf_path, preview_sdf_new, check=args.check):
        changed.append(preview_sdf_path)
    if _process_file(controlled_sdf_path, controlled_sdf_new, check=args.check):
        changed.append(controlled_sdf_path)

    if args.check:
        if changed:
            print("lr_drone model files are out of sync with lr_drone_alignment.yaml:")
            for path in changed:
                print(f"  - {path}")
            return 1
        print("lr_drone model files match lr_drone_alignment.yaml")
        return 0

    if changed:
        print("Updated lr_drone model files from alignment YAML:")
        for path in changed:
            print(f"  - {path}")
    else:
        print("No changes: lr_drone model files already matched alignment YAML")
    return 0


if __name__ == "__main__":
    sys.exit(main())
