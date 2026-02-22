"""Interactive Matplotlib teleop mode for the standalone simulator."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from ..core.config import NormalizedSimConfig
from ..core.registry import create_backend, register_builtin_components
from ..core.types import ControlTarget, SimState
from ..terrain_wrapper import (
    HeightFieldTerrain,
    generate_terrain,
    is_point_in_collision,
    load_terrain_config,
)

logger = logging.getLogger(__name__)


@dataclass
class _TeleopKeys:
    pressed: set[str] = field(default_factory=set)
    paused: bool = False
    running: bool = True

    def _normalize(self, key: str | None) -> str:
        if key is None:
            return ""
        # Matplotlib may report combos like "shift+w".
        return str(key).lower().split("+")[-1].strip()

    def on_press(self, key: str | None) -> None:
        k = self._normalize(key)
        if not k:
            return
        if k in {"escape"}:
            self.running = False
            return
        if k in {"p"}:
            self.paused = not self.paused
            return
        if k in {"x"}:
            # Emergency stop of user command (kept one-shot; not a latched key).
            self.pressed.discard("w")
            self.pressed.discard("a")
            self.pressed.discard("s")
            self.pressed.discard("d")
            self.pressed.discard("r")
            self.pressed.discard("f")
            return
        self.pressed.add(k)

    def on_release(self, key: str | None) -> None:
        k = self._normalize(key)
        if k:
            self.pressed.discard(k)


def _compute_max_tree_top(obstacles: list[Any]) -> float:
    max_tree_top = 0.0
    for o in obstacles:
        if hasattr(o, "heights"):
            top = float(np.max(getattr(o, "heights")))
        else:
            c = o.center
            top = float(c[2])
            if hasattr(o, "height"):
                top += float(getattr(o, "height"))
            elif hasattr(o, "size"):
                top += float(getattr(o, "size")[2]) / 2.0
        if top > max_tree_top:
            max_tree_top = top
    return max_tree_top


def _draw_scene(
    ax: Any,
    *,
    obstacles: list[Any],
    terrain_type: str,
    space_dim: np.ndarray,
    visual_cfg: dict[str, Any],
) -> None:
    tree_linewidth = float(visual_cfg.get("tree_linewidth", 4.0))
    tree_alpha = float(visual_cfg.get("tree_alpha", 0.9))
    tree_radius_ref = float(visual_cfg.get("tree_radius_ref", 1.0))
    terrain_alpha = float(visual_cfg.get("terrain_alpha", 0.45))
    terrain_cmap = str(visual_cfg.get("terrain_cmap", "terrain"))

    if terrain_type == "forest":
        for o in obstacles:
            c = o.center
            z0 = float(c[2])
            if hasattr(o, "height"):
                h = float(getattr(o, "height"))
            elif hasattr(o, "size"):
                h = float(getattr(o, "size")[2])
            else:
                h = 2.0
            radius = float(getattr(o, "radius", tree_radius_ref))
            radius_scale = max(0.4, min(3.0, radius / max(tree_radius_ref, 1e-6)))
            ax.plot(
                [float(c[0]), float(c[0])],
                [float(c[1]), float(c[1])],
                [z0, z0 + h],
                color="green",
                linewidth=tree_linewidth * radius_scale,
                alpha=tree_alpha,
            )
    elif terrain_type == "mountains":
        terrain = next((o for o in obstacles if isinstance(o, HeightFieldTerrain)), None)
        if terrain is not None:
            X, Y = np.meshgrid(terrain.xs, terrain.ys, indexing="ij")
            ax.plot_surface(
                X,
                Y,
                terrain.heights,
                cmap=terrain_cmap,
                linewidth=0.0,
                antialiased=True,
                alpha=terrain_alpha,
                zorder=1,
            )
        cylinders = [o for o in obstacles if hasattr(o, "radius")]
        if cylinders:
            obs_x = [float(c.center[0]) for c in cylinders]
            obs_y = [float(c.center[1]) for c in cylinders]
            obs_z = [float(c.center[2]) + float(getattr(c, "height", 0.0)) / 2.0 for c in cylinders]
            ax.scatter(obs_x, obs_y, obs_z, color="gray", alpha=0.6, s=10, zorder=2)
    else:
        obs_x: list[float] = []
        obs_y: list[float] = []
        obs_z: list[float] = []
        for o in obstacles:
            c = o.center
            obs_x.append(float(c[0]))
            obs_y.append(float(c[1]))
            zc = float(c[2])
            if hasattr(o, "height"):
                zc += float(getattr(o, "height")) / 2.0
            elif hasattr(o, "size"):
                zc += float(getattr(o, "size")[2]) / 2.0
            obs_z.append(zc)
        if obs_x:
            ax.scatter(obs_x, obs_y, obs_z, color="gray", alpha=0.6, s=12)

    ax.set_xlim(0.0, float(space_dim[0]))
    ax.set_ylim(0.0, float(space_dim[1]))
    ax.set_zlim(0.0, float(space_dim[2]))
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.grid(True)
    ax.view_init(elev=30, azim=135)


def run_teleop_session(cfg_norm: NormalizedSimConfig) -> None:
    """Run interactive keyboard teleop in a Matplotlib 3D view."""
    if cfg_norm.seed is not None:
        np.random.seed(cfg_norm.seed)

    path_cfg = dict(cfg_norm.path_cfg)
    ctrl_cfg = dict(cfg_norm.controller_cfg)
    vis_cfg = dict(cfg_norm.visual_cfg)

    if cfg_norm.terrain_config_path is not None:
        terrain_cfg = load_terrain_config(Path(cfg_norm.terrain_config_path))
    else:
        terrain_cfg = load_terrain_config()
    if cfg_norm.terrain_override is not None:
        terrain_cfg.terrain_type = cfg_norm.terrain_override

    obstacles = generate_terrain(
        terrain_cfg,
        forest_density_scale=float(vis_cfg.get("forest_density_scale", 1.0)),
        tree_height_scale=float(vis_cfg.get("tree_height_scale", 1.0)),
    )
    terrain = next((o for o in obstacles if hasattr(o, "height_at")), None)
    max_tree_top = _compute_max_tree_top(obstacles)

    if terrain_cfg.terrain_type in {"forest", "mountains"} and max_tree_top > 0.0:
        height_ratio = float(vis_cfg.get("height_ratio", 1.2))
        terrain_cfg.space_dim = terrain_cfg.space_dim.copy()
        terrain_cfg.space_dim[2] = max_tree_top * height_ratio

    max_z_allowed = min(max_tree_top if max_tree_top > 0.0 else float(terrain_cfg.space_dim[2]), float(terrain_cfg.space_dim[2]))

    sx_rel = float(path_cfg.get("start_relative_x", 0.02))
    sy_rel = float(path_cfg.get("start_relative_y", 0.02))
    sz_rel = float(path_cfg.get("start_relative_z", 0.05))
    start_pos = np.array(
        [
            sx_rel * float(terrain_cfg.space_dim[0]),
            sy_rel * float(terrain_cfg.space_dim[1]),
            sz_rel * float(terrain_cfg.space_dim[2]),
        ],
        dtype=float,
    )

    terrain_clearance = float(path_cfg.get("terrain_clearance", 2.0))
    if terrain is not None and hasattr(terrain, "height_at"):
        ground = float(terrain.height_at(float(start_pos[0]), float(start_pos[1])))
        start_pos[2] = max(start_pos[2], ground + terrain_clearance)

    register_builtin_components()
    backend_name = str(cfg_norm.backend_name).lower()
    if backend_name != "rotorpy":
        logger.warning(
            "Teleop works best with RotorPy backend. Current backend=%s (use --backend rotorpy).",
            backend_name,
        )
    backend = create_backend(backend_name)
    backend.reset(
        initial_state=SimState(position=start_pos.copy(), velocity=np.zeros(3, dtype=float), t=0.0),
        world={
            "space_dim": terrain_cfg.space_dim.copy(),
            "max_z_allowed": max_z_allowed,
            "terrain": terrain,
            "terrain_clearance": terrain_clearance,
        },
        cfg={
            "controller": ctrl_cfg,
            "simulation": cfg_norm.simulation_cfg,
        },
    )

    dt = float(cfg_norm.dt)
    sim_time_limit = float(cfg_norm.sim_time)
    teleop_cfg = dict(ctrl_cfg.get("teleop", {}) or {})
    accel_step_xy = float(teleop_cfg.get("accel_step_xy", 3.0))
    accel_step_z = float(teleop_cfg.get("accel_step_z", 3.0))
    vel_damp = float(teleop_cfg.get("vel_damp", 1.5))
    max_speed_xy = float(teleop_cfg.get("max_speed_xy", 6.0))
    max_speed_z = float(teleop_cfg.get("max_speed_z", 4.0))
    acc_max = float(ctrl_cfg.get("acc_max", 20.0))

    logger.info("Teleop controls: W/S (+/-X), A/D (+/-Y), R/F (+/-Z), P pause, Esc close")
    logger.info("Starting teleop (%s backend), dt=%.3fs, time_limit=%.1fs", backend_name, dt, sim_time_limit)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    _draw_scene(
        ax,
        obstacles=list(obstacles),
        terrain_type=str(terrain_cfg.terrain_type),
        space_dim=np.asarray(terrain_cfg.space_dim, dtype=float),
        visual_cfg=vis_cfg,
    )
    ax.set_title(f"{str(terrain_cfg.terrain_type).capitalize()} teleop [{backend_name.upper()}]")

    state0 = backend.state()
    trail_positions: list[np.ndarray] = [state0.position.copy()]

    trail_line, = ax.plot(
        [state0.position[0]],
        [state0.position[1]],
        [state0.position[2]],
        color=str(vis_cfg.get("path_color", "deepskyblue")),
        linewidth=float(vis_cfg.get("path_linewidth", 2.0)),
        label="Flight path",
        zorder=5,
    )
    drone_marker = ax.scatter(
        [state0.position[0]],
        [state0.position[1]],
        [state0.position[2]],
        color="cyan",
        s=float(vis_cfg.get("marker_size", 40.0)),
        label="Drone",
        zorder=6,
    )
    ax.legend(loc="upper right")

    status_text = fig.text(
        0.02,
        0.02,
        "",
        fontsize=9,
        family="monospace",
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.7"},
    )

    keys = _TeleopKeys()
    min_bounds = np.array([0.0, 0.0, 0.0], dtype=float)
    max_bounds = np.array(
        [float(terrain_cfg.space_dim[0]), float(terrain_cfg.space_dim[1]), float(max_z_allowed)],
        dtype=float,
    )

    def _manual_accel_from_keys(state: SimState) -> np.ndarray:
        cmd = np.zeros(3, dtype=float)
        if "w" in keys.pressed:
            cmd[0] += accel_step_xy
        if "s" in keys.pressed:
            cmd[0] -= accel_step_xy
        if "d" in keys.pressed:
            cmd[1] += accel_step_xy
        if "a" in keys.pressed:
            cmd[1] -= accel_step_xy
        if "r" in keys.pressed:
            cmd[2] += accel_step_z
        if "f" in keys.pressed:
            cmd[2] -= accel_step_z

        # Add damping so releasing keys tends to settle the drone.
        cmd -= vel_damp * np.asarray(state.velocity, dtype=float)

        v = np.asarray(state.velocity, dtype=float)
        horiz_speed = float(np.linalg.norm(v[:2]))
        if horiz_speed > max_speed_xy and horiz_speed > 1e-6:
            cmd[:2] -= (v[:2] / horiz_speed) * vel_damp * (horiz_speed - max_speed_xy)
        if abs(float(v[2])) > max_speed_z:
            cmd[2] -= np.sign(v[2]) * vel_damp * (abs(float(v[2])) - max_speed_z)

        mag = float(np.linalg.norm(cmd))
        if mag > acc_max > 1e-6:
            cmd *= acc_max / mag
        return cmd

    def _refresh_artists(state: SimState) -> None:
        pos = np.asarray(state.position, dtype=float)
        trail_arr = np.vstack(trail_positions)
        trail_line.set_data_3d(trail_arr[:, 0], trail_arr[:, 1], trail_arr[:, 2])
        # 3D scatter update uses private-ish attr but is standard in mpl3d.
        drone_marker._offsets3d = ([pos[0]], [pos[1]], [pos[2]])  # type: ignore[attr-defined]

        colliding = bool(is_point_in_collision(pos, obstacles, inflation=0.0))
        speed = float(np.linalg.norm(state.velocity))
        status_text.set_text(
            f"t={state.t:6.2f}s  pos=[{pos[0]:6.1f},{pos[1]:6.1f},{pos[2]:5.1f}]  "
            f"speed={speed:4.2f} m/s  paused={keys.paused}  collision={colliding}\n"
            "Keys: W/S X, A/D Y, R/F Z, P pause, Esc close"
        )
        drone_marker.set_color("red" if colliding else "cyan")

    timer = fig.canvas.new_timer(interval=max(1, int(dt * 1000.0)))

    def _on_tick() -> None:
        if not plt.fignum_exists(fig.number):
            return
        if not keys.running:
            plt.close(fig)
            return

        state = backend.state()
        if not keys.paused:
            acc_cmd = _manual_accel_from_keys(state)
            backend.step(ControlTarget(accel_cmd=acc_cmd, metadata={"controller": "teleop"}), dt)
            backend.apply_constraints(
                min_bounds=min_bounds,
                max_bounds=max_bounds,
                terrain=terrain,
                terrain_clearance=terrain_clearance,
            )
            state = backend.state()
            trail_positions.append(np.asarray(state.position, dtype=float).copy())

        _refresh_artists(state)
        fig.canvas.draw_idle()

        if sim_time_limit > 0.0 and float(state.t) >= sim_time_limit:
            logger.info("Teleop session reached time limit (%.1fs). Closing window.", sim_time_limit)
            plt.close(fig)

    timer.add_callback(_on_tick)

    def _on_key_press(event: Any) -> None:
        keys.on_press(getattr(event, "key", None))

    def _on_key_release(event: Any) -> None:
        keys.on_release(getattr(event, "key", None))

    def _on_close(_event: Any) -> None:
        keys.running = False
        try:
            timer.stop()
        except Exception:
            pass

    fig.canvas.mpl_connect("key_press_event", _on_key_press)
    fig.canvas.mpl_connect("key_release_event", _on_key_release)
    fig.canvas.mpl_connect("close_event", _on_close)

    _refresh_artists(state0)
    timer.start()
    plt.tight_layout()
    plt.show()

