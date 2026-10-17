import sys
from pathlib import Path

import pytest

# The demo scene lives with the examples.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples" / "planning"))

import scene  # noqa: E402


@pytest.fixture(scope="session")
def room():
    """The demo scene point cloud (examples/planning/scene.py)."""
    return scene.room_pointcloud()
