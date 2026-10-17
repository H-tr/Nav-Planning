#!/usr/bin/env bash
# Spherize each robot's URDF with foam, then cut the base model out of the
# spherized URDF (so both collision models share the same spheres) and
# finish the SRDF (scripts/build_robot_description.py spheres).
#
# foam's default caps a mesh at 8 spheres, which leaves boxy links such as
# the Ridgeback chassis 0.39 m bigger than their mesh.  Up to 32 spheres a
# mesh, with the volume heuristic scaled to match, brings that to 0.13 m;
# scripts/sphere_coverage.py measures the fit.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

RESOURCES="$ROOT/nav_planning/resources/robot"

for ROBOT in locobot ridgeback; do
    python "$ROOT/third_party/foam/scripts/generate_sphere_urdf.py" \
        "$RESOURCES/$ROBOT/$ROBOT.urdf" \
        --output "$RESOURCES/$ROBOT/${ROBOT}_spherized.urdf" \
        --database "$ROOT/third_party/foam/sphere_database.json" \
        --branch 32 \
        --volume_heuristic_ratio 8 \
        "$@"
    python "$SCRIPT_DIR/build_robot_description.py" spheres "$RESOURCES" "$ROBOT"
done
