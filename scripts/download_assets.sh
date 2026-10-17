#!/usr/bin/env bash
# Download the official robot descriptions that scripts/build_robot.sh
# turns into the planning descriptions under nav_planning/resources/robot/.
# Each repository is pinned to the commit the shipped descriptions were
# built from.  The planning descriptions are already in the package, so
# this is only needed to rebuild them.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
ASSETS="$ROOT/assets/descriptions"

fetch() {  # fetch <dir> <url> <commit>
    local dest="$ASSETS/$1"
    if [ ! -d "$dest/.git" ]; then
        git init -q "$dest"
        git -C "$dest" remote add origin "$2"
    fi
    git -C "$dest" fetch -q --depth 1 origin "$3"
    git -C "$dest" checkout -q FETCH_HEAD
    echo "  $1 @ $3"
}

echo "Downloading official descriptions into $ASSETS ..."
mkdir -p "$ASSETS"
# LoCoBot (ROS 2 Humble branch, iRobot Create 3 base)
fetch interbotix_ros_rovers https://github.com/Interbotix/interbotix_ros_rovers.git 25a1513fc07e38cf081d986fa3be9afa0ad5413c
fetch create3_sim https://github.com/iRobotEducation/create3_sim.git 7d7a4f69a503d4eef22f13aeb8755c86955b8cc5
# Clearpath Ridgeback, FLIR PTU-5 and Intel RealSense D435
fetch ridgeback https://github.com/ridgeback/ridgeback.git 7f2489993f1369906e808402e8f45e4c2aebf5b3
# Included (not used) by ridgeback_description's laser accessories
fetch LMS1xx https://github.com/clearpathrobotics/LMS1xx.git 1dc281708ef0d9d69f2e58eeba3cb694ed150422
fetch flir_ptu https://github.com/ros-drivers/flir_ptu.git 7602f01fb7fe15141d72c7f58275afbc8441c01d
fetch realsense-ros https://github.com/realsenseai/realsense-ros.git 7a87097c499e3b0b69518d54a8e7a9ac1924e87c
echo "Done."
