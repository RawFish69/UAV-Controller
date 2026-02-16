"""Standalone Python UAV simulator package.

Architecture highlights:
- Plugin-style registries for planners, controllers, and dynamics backends
- Backward-compatible CLI entrypoint via ``python -m sim_py.run_sim``
- Optional RotorPy backend with import guards
"""
