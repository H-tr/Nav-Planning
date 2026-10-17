#!/usr/bin/env bash
# Build the planning robot descriptions (URDF, base URDF, SRDF, meshes)
# from the official descriptions downloaded by scripts/download_assets.sh.
# All project-specific paths live here; the Python script is the tool.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

python "$SCRIPT_DIR/build_robot_description.py" description \
    "$ROOT/assets/descriptions" \
    "$ROOT/nav_planning/resources/robot" \
    "$@"
