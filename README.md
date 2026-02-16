# UAV Controller + Path Planning

Multi-purpose quadcopter control stack with:
- **ROS 2 control + safety pipeline** (hardware and RViz simulation)
- **Standalone Python simulator** (no ROS) for fast iteration on planners/controllers
- **ESP32 TX/RX link** for manual flight + autonomous command relay

## What’s in this repo

- **Controllers**
  - **ROS 2**: PID / LQR / MPC (work-in-progress depending on package)
  - **Python-only** (`sim_py`): PID / LQR / MPC position controllers
- **Path planning (Python-only)**: straight / A* / RRT planners
- **Terrain generation**: forest / mountains / plains (shared between ROS and Python sim)
- **Safety**: validation, limiting, watchdog (`safety_gate`)
- **Hardware link**: CRSF adapter + ESP-NOW based TX/RX + protocol bridging

## Demo (A*, RRT, RRT* path planner)

<p>
  <img src="docs/mountain_astar.png" alt="Mountain path planner with A*" width="49%">
  <img src="docs/mountain_rrt.png" alt="Mountain path planner with RRT" width="49%">
</p>

<p>
  <img src="docs/mountain_rrt_star_0.png" alt="Mountain path planner with RRT*" width="49%">
  <img src="docs/mountain_rrt_star_1.png" alt="Forest path planner with RRT*" width="49%">
</p>

## Architecture (high level)

**Manual Flight**

```
TX (IMU+Joystick) → ESP-NOW → RX → Protocol Bridge → Flight Controller
                                      (CRSF/SBUS/PPM/iBus/FrSky)
```

**Autonomous (Hardware-in-the-loop)**

```
ROS Controllers → Safety Gate → CRSF Adapter → UDP → TX → ESP-NOW → RX → Protocol → FC
```

**Simulation**

- **ROS 2 + RViz**:

```
Controllers → Safety Gate → sim_dyn → RViz
```

- **Python-only (no ROS)**:

```
Planner → Controller → Point-mass dynamics → Matplotlib 3D
```

## Quick start (Python-only simulator)

```bash
pip install -r sim_py/requirements.txt
python -m sim_py.run_sim
```

Useful overrides:

```bash
# Switch controller
python -m sim_py.run_sim --controller mpc

# Change terrain type (still uses the terrain config YAML unless overridden)
python -m sim_py.run_sim --terrain forest

# Override sim time / dt (if you pass these, they override sim_config.yaml)
python -m sim_py.run_sim --sim-time 240 --dt 0.01

# Use a different terrain config file
python -m sim_py.run_sim --terrain-config ros2_ws/src/terrain_generator/config/terrain_params.yaml
```

### Python sim configuration

- **Main config**: `sim_py/sim_config.yaml`
  - **Start/goal**: `path.start_relative_*`, `path.end_relative_*`
    - `end_relative_z: "auto"` picks a random goal altitude in \([0, \text{tallest tree}]\)
  - **Planner**: `path.planner_type` = `straight` | `astar` | `rrt`
  - **Runtime**: `controller.sim_time`, `controller.dt`
  - **Terrain appearance / scaling**:
    - `visual.forest_density_scale`: scales forest density (clamped to 1.0)
    - `visual.tree_height_scale`: scales sampled tree heights
    - `visual.height_ratio`: sets map height as `height_ratio * tallest_tree`
    - `visual.tree_radius_ref`: reference radius for drawing thicker/thinner trunks

- **Terrain config** (shared with ROS):
  - `ros2_ws/src/terrain_generator/config/terrain_params.yaml`
  - Forest obstacle count is mainly set by:
    - `forest.grid_size` and `forest.density`
    - expected trees ≈ \(grid\_size^2 \cdot density\)

## Quick start (ROS 2)

### Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run RViz simulation

```bash
ros2 launch sim_dyn sim_pid.launch.py
ros2 launch sim_dyn sim_lqr.launch.py
ros2 launch sim_dyn sim_mpc.launch.py
```

### Run autonomous (hardware)

TX firmware needs UDP modification. See `docs/HARDWARE.md`.

```bash
./scripts/run_crsf_link_pid.sh transport:=udp udp_host:=192.168.4.1
```

## GPS module (ESP32)

The `gps/` project is an ESP32 GPS bring-up/telemetry module using `Adafruit_GPS`.

- Supports PMTK/NMEA modules (Adafruit Ultimate GPS / MTK33xx style)
- Supports u-blox modules with UBX configuration (while parsing NMEA output)
- Auto-probes common UART baud rates (9600/38400/115200), parses fix/satellite/SNR metrics, and prints diagnostics over serial
- Build/flash protocol options:
  - `pio run -d gps -e gps_auto -t upload` (AUTO detect PMTK vs UBLOX)
  - `pio run -d gps -e gps_pmtk -t upload` (force PMTK mode)
  - `pio run -d gps -e gps_ublox -t upload` (force UBLOX mode)

<img src="docs/gps_demo_1.png" alt="GPS telemetry demo output" width="700">

## Packages / folders

| Path | Type | Purpose |
|------|------|---------|
| `ros2_ws/src/controllers_pid` | C++ | Cascaded PID controller |
| `ros2_ws/src/controllers_lqr` | C++ | LQR controller |
| `ros2_ws/src/controllers_mpc` | C++ | MPC controller |
| `ros2_ws/src/safety_gate` | C++ | Validation, limiting, routing |
| `ros2_ws/src/adapters_crsf` | Python | ROS ↔ TX bridge (UDP/Serial) |
| `ros2_ws/src/sim_dyn` | Python | Dynamics + RViz integration |
| `ros2_ws/src/terrain_generator` | Python | Terrain + obstacles (forest/mountains/plains) |
| `ros2_ws/src/common_msgs` | ROS msgs | Custom message types |
| `sim_py` | Python | Standalone planner/controller/dynamics/visualization |
| `ESPNOW_TX` | ESP32 | ESP-NOW based TX/RX firmware |
| `LoRa_TX` | ESP32 | LoRa TX experiments |
| `gps` | ESP32 | GPS telemetry module (Adafruit_GPS / NMEA + PMTK + UBX support) |
| `Utils` | Python | Protocol decoder/monitor + tools |

## ROS topics (common)

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd/body_rate_thrust` | `BodyRateThrust` | Controller output |
| `/cmd/final/body_rate_thrust` | `BodyRateThrust` | After safety (sim) |
| `/cmd/final/rc` | `VirtualRC` | After safety (hardware) |
| `/state/odom` | `Odometry` | State estimate |
| `/state/attitude` | `QuaternionStamped` | Orientation |
| `/state/angular_velocity` | `Vector3Stamped` | Body rates |

## Docs

- **[docs/SOFTWARE_GUIDE.md](docs/SOFTWARE_GUIDE.md)**: software guide (start here)
- **[docs/EXAMPLE_USAGE.md](docs/EXAMPLE_USAGE.md)**: terrain + controller examples
- **`docs/HARDWARE.md`**: TX integration for autonomous mode
- **`ESPNOW_TX/README.md`**: TX/RX firmware details
- **`gps/DASHBOARD.md`**: GPS serial dashboard usage
- **`Utils/README.md`**: protocol monitor / decoder tooling

## Notes

- **ROS 2 Humble** is required for ROS-based control + RViz simulation
- The **Python-only sim** (`sim_py`) is designed for fast iteration (no ROS needed)

## Hardware photo

<img src="docs/brushed.jpg" alt="Brushed Motor Quadcopter" width="400">

*Test brushed quad with the custom TX/RX.*
