from __future__ import annotations

import sys
from pathlib import Path


def ensure_repo_root_on_path() -> Path:
    """Add repo root to sys.path for importing `sim_py` during source builds."""
    here = Path(__file__).resolve()
    repo_root = here.parents[4]  # .../UAV-Controller
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    return repo_root
