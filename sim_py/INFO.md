# sim_py Information

## What `sim_py` is

`sim_py` is the standalone Python simulation stack in this repo.

It is organized as a lightweight framework with pluggable components:

- `Planner`: builds waypoints from start to goal
- `Controller`: computes control targets from state + waypoint
- `DynamicsBackend`: advances simulation state over time

Core orchestration is in `sim_py/core/runner.py`.

## Quick usage

From repo root:

```bash
./scripts/setup_sim_py_venv.sh
source sim_py/.venv/bin/activate
python -m sim_py.run_sim
```

Useful options:

```bash
# Switch controller
python -m sim_py.run_sim --controller lqr

# Override terrain
python -m sim_py.run_sim --terrain mountains

# Override runtime
python -m sim_py.run_sim --sim-time 120 --dt 0.01

# Optional RotorPy backend
./scripts/setup_sim_py_venv.sh --with-rotorpy
source sim_py/.venv/bin/activate
python -m sim_py.run_sim --backend rotorpy

# RotorPy + mountains terrain
python -m sim_py.run_sim --backend rotorpy --terrain mountains

# RotorPy + mountains + LQR controller
python -m sim_py.run_sim --backend rotorpy --terrain mountains --controller lqr
```

Headless environments:

```bash
MPLBACKEND=Agg python -m sim_py.run_sim --backend pointmass
```

## Main config

Primary config file: `sim_py/sim_config.yaml`

Key sections:

- `path.*`: planner config and start/goal settings
- `controller.*`: controller selection and gains
- `visual.*`: plotting and terrain visual scaling
- `simulation.backend`: `pointmass` (default) or `rotorpy`
- `simulation.seed`: optional deterministic random seed
- `simulation.rotorpy.*`: optional RotorPy backend settings

CLI flags override config values where applicable.

## Architecture map

Entry + app:

- `sim_py/run_sim.py` (compat entrypoint)
- `sim_py/app/cli.py`
- `sim_py/app/main.py`

Core framework:

- `sim_py/core/types.py`
- `sim_py/core/interfaces.py`
- `sim_py/core/registry.py`
- `sim_py/core/config.py`
- `sim_py/core/runner.py`

Built-in plugins:

- `sim_py/planners/basic.py`
- `sim_py/controllers/basic.py`
- `sim_py/backends/pointmass_backend.py`
- `sim_py/backends/rotorpy_backend.py`

## Built-in registry keys

Planners:

- `straight`
- `astar`
- `rrt`
- `rrtstar` (aliases: `rrt*`, `rrt_star`)

Controllers:

- `pid`
- `lqr`
- `mpc`

Backends:

- `pointmass`
- `rotorpy`

## How to add new plugins

1. Implement one interface from `sim_py/core/interfaces.py`.
2. Register it in `sim_py/core/registry.py`.
3. Keep config namespaced:
- planner params under `path.*`
- controller params under `controller.<name>.*`
- backend params under `simulation.<backend_name>.*`

No runner changes are required for normal planner/controller/backend additions.
