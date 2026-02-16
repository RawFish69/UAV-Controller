"""Backward-compatible entrypoint for the standalone simulator.

Usage (from repo root):

    pip install -r sim_py/requirements.txt
    python -m sim_py.run_sim --terrain forest --controller pid
"""

from __future__ import annotations

from .app.main import main


if __name__ == "__main__":
    main()
