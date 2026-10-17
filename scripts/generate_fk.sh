#!/usr/bin/env bash
# Generate FK/collision-checking C++ code for vamp using cricket.
# Generates four VAMP robot models and installs them in
# ext/ompl_vamp/include/vamp/robots/ (NOT inside the vamp submodule):
#
#   - LocobotWholeBody    (5 DOF)  from locobot_spherized.urdf + locobot.srdf
#   - LocobotBase         (3 DOF)  from locobot_base.urdf + locobot_base.srdf
#   - RidgebackWholeBody  (5 DOF)  from ridgeback_spherized.urdf + ridgeback.srdf
#   - RidgebackBase       (3 DOF)  from ridgeback_base.urdf + ridgeback_base.srdf
#
# The base models hold the links that do not move with the camera, cut
# from the spherized URDF (scripts/spherize_robot.sh).  They are the
# collision model of the multilevel planners' base level.
#
# cricket's CppAD codegen is not bit-stable across runs: regenerating
# can move spheres by ~1e-6 m.  Regenerate only when a URDF or SRDF
# actually changes.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

CRICKET="$ROOT/third_party/cricket"
RESOURCES="$ROOT/nav_planning/resources/robot"
CRICKET_CONFIGS="$SCRIPT_DIR/cricket_configs"
FK_DEST="$ROOT/ext/ompl_vamp/include/vamp/robots"

# 1. Distribute robot descriptions and cricket configs into the cricket
#    submodule (kept out of submodule git tracking on purpose — the
#    canonical copies live in this repo).
echo "[1/3] Distributing robot descriptions..."
for ROBOT in locobot ridgeback; do
    mkdir -p "$CRICKET/resources/$ROBOT"
    cp "$RESOURCES/$ROBOT/${ROBOT}_spherized.urdf" "$CRICKET/resources/$ROBOT/"
    cp "$RESOURCES/$ROBOT/$ROBOT.srdf"             "$CRICKET/resources/$ROBOT/"
    cp "$RESOURCES/$ROBOT/${ROBOT}_base.urdf"      "$CRICKET/resources/$ROBOT/"
    cp "$RESOURCES/$ROBOT/${ROBOT}_base.srdf"      "$CRICKET/resources/$ROBOT/"
done
cp "$CRICKET_CONFIGS/"*.json "$CRICKET/resources/"

# 2. Run cricket FK code generation
echo "[2/3] Generating FK code..."
mkdir -p "$FK_DEST"
for MODEL in locobot_whole_body locobot_base ridgeback_whole_body ridgeback_base; do
    echo "  generating FK for $MODEL..."
    (cd "$CRICKET" && "$CRICKET/build/fkcc_gen" "resources/${MODEL}.json")
    cp "$CRICKET/${MODEL}_fk.hh" "$FK_DEST/${MODEL}.hh"
    echo "    installed $FK_DEST/${MODEL}.hh"
done

echo "[3/3] Done! Rebuild the package to use the new FK code."
