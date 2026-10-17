# Getting Started

## Prerequisites

- **Linux** (x86_64, glibc 2.28+) on a CPU with AVX2 (Intel Haswell / AMD Zen or newer)
- **Python** 3.10–3.14

## Installation

Pre-built wheels are available for Python 3.10–3.14 on Linux x86_64. No local compilation required:

```bash
pip install nav-planning
```

## Verify installation

```python
from nav_planning.locobot import HOME_JOINTS
from nav_planning.planning import create_planner

planner = create_planner("locobot")
goal = planner.sample_valid()
result = planner.plan(HOME_JOINTS.copy(), goal)
print(f"Planning {'succeeded' if result.success else 'failed'}")
```

## Building Wheels from Source

Wheels are built with [cibuildwheel](https://cibuildwheel.pypa.io) inside
the manylinux Docker images (configured under `[tool.cibuildwheel]` in
`pyproject.toml`). To build one locally:

```bash
pipx run cibuildwheel --platform linux --only cp312-manylinux_x86_64
```

The output goes to `wheelhouse/`. Requirements: Docker must be installed and running.

## Releasing

Pushing a `v*` tag runs `.github/workflows/release.yml`, which builds
manylinux_2_34 and manylinux_2_28 wheels for every supported Python and
publishes them to PyPI via Trusted Publishing (project `nav-planning`,
GitHub environment `pypi`):

```bash
# after bumping `version` in pyproject.toml on main
git tag v0.1.0
git push origin v0.1.0
```

## Development Setup

For contributing or rebuilding the robot models, use [pixi](https://pixi.sh):

```bash
git clone --recursive https://github.com/H-tr/Nav-Planning.git
cd Nav-Planning
bash scripts/setup.sh
```

Or manually:

```bash
git clone --recursive https://github.com/H-tr/Nav-Planning.git
cd Nav-Planning
pixi install          # also builds nav_planning into the environment
pixi run test
```

### Rebuilding the robot models

The collision models ship with the package.  To rebuild them from the
official descriptions (see [Robots](robots.md)):

```bash
bash scripts/download_assets.sh     # official descriptions, pinned commits
pixi run cricket-build
pixi run generate-fk                # build-robot -> spherize-robot -> generate-fk
pixi run python scripts/sphere_coverage.py nav_planning/resources/robot locobot
```

`generate-fk` depends on `spherize-robot`, which depends on
`build-robot`, so one call runs the whole pipeline.  Rebuild the package
afterwards (`pixi install`).
