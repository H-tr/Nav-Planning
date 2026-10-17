"""The Ridgeback's bundled description.

This module collects every concrete value that describes the Ridgeback:
joint groupings, the home pose, the planning subgroups, the kinodynamic
limits and the top-level ``RobotConfig`` instance.

The robot is the Clearpath Ridgeback omnidirectional base from the
official ``ridgeback_description`` (github.com/ridgeback/ridgeback,
``melodic-devel``) without payload accessories.  The Ridgeback has no
head; Nav-Planning adds one (``scripts/ridgeback_camera_mast.urdf.xacro``):

* a 40 x 40 mm aluminium extrusion, 0.70 m tall, on the centre of the
  payload deck (the description's ``mid_mount`` frame, 0.306 m above the
  floor);
* a FLIR PTU-5 pan-tilt unit on top of the mast — its base 1.006 m above
  the floor (official ``flir_ptu_description``, github.com/ros-drivers/
  flir_ptu, model ``ptu5``);
* an Intel RealSense D435 on the PTU's payload mount (official
  ``realsense2_description``, github.com/realsenseai/realsense-ros).

The full configuration vector is 5-DOF:

* ``[0:3]`` — mobile base ``(x, y, theta)``, holonomic (mecanum wheels)
* ``[3:5]`` — PTU ``ptu_pan`` and ``ptu_tilt``

The dataclass *types* live in :mod:`nav_planning.types.robot`; this file
holds *values* of those types.
"""

import math
import os

import numpy as np

from nav_planning.types.robot import CameraConfig, RobotConfig

_PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
_RESOURCES_DIR = os.path.join(_PKG_ROOT, "resources", "robot", "ridgeback")

# The C++ planner's name for this robot.
ROBOT_NAME = "ridgeback"

# Atomic joint groups — indices into the full 5-DOF configuration array.
# Order matches VAMP's URDF tree traversal: base → pan → tilt.
JOINT_GROUPS = {
    "base": slice(0, 3),  # base_x, base_y, base_theta (holonomic)
    "camera": slice(3, 5),  # ptu_pan, ptu_tilt
}

# Kinodynamic limits used by ``MotionPlanner.plan_kinodynamic`` (FLASK).
#
# FLIR PTU-5 user manual (v3.10, 2020), section 1.2: "Axis Speeds:
# 120°/second pan; 50°/second tilt" (the 2017 datasheet lists 150°/s pan;
# we take the lower figure).  The joint ranges are the manual's default
# user limits (section 5.1.1), +-168 deg pan and -90/+30 deg tilt, written
# into the URDF by scripts/build_robot_description.py.  FLIR publishes no
# acceleration ("Acceleration/Deceleration Programmable"); we assume each
# axis reaches its top speed in one second — set these to the
# accelerations programmed on your PTU.
JOINT_VELOCITY_LIMITS: dict[str, float] = {
    "ptu_pan": math.radians(120.0),
    "ptu_tilt": math.radians(50.0),
}
JOINT_ACCELERATION_LIMITS: dict[str, float] = {
    "ptu_pan": math.radians(120.0),
    "ptu_tilt": math.radians(50.0),
}

# Ridgeback velocity controller limits (ridgeback_control/config/
# control.yaml): linear x and y "max_velocity: 1.1 # m/s" and
# "max_acceleration: 2.5 # m/s^2" per body axis, angular z
# "max_velocity: 2.0 # rad/s" and "max_acceleration: 1.0 # rad/s^2".
# The Ridgeback data sheet (May 2025) also gives "MAX SPEED 1.1 m/s".
BASE_MAX_SPEED: float = 1.1  # m/s, each body axis
BASE_MAX_ACCELERATION: float = 2.5  # m/s^2, each body axis
BASE_MAX_YAW_RATE: float = 2.0  # rad/s
BASE_MAX_YAW_ACCELERATION: float = 1.0  # rad/s^2

VIZ_URDF_PATH = os.path.join(_RESOURCES_DIR, "ridgeback.urdf")

ridgeback_robot_config = RobotConfig(
    urdf_path=os.path.join(_RESOURCES_DIR, "ridgeback.urdf"),
    joint_names=[
        # [0:3]   mobile base (virtual planar joints)
        "base_x_joint",
        "base_y_joint",
        "base_theta_joint",
        # [3:5]   pan-tilt unit
        "ptu_pan",
        "ptu_tilt",
    ],
    # RealSense D435 colour stream, 69 x 42 deg field of view
    # (Intel RealSense D400 series datasheet).
    camera=CameraConfig(
        link_name="camera_color_optical_frame",
        width=640,
        height=360,
        fov=42.0,
        near=0.1,
        far=10.0,
    ),
)

_BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_theta_joint"]
_CAMERA_JOINTS = ["ptu_pan", "ptu_tilt"]

PLANNING_SUBGROUPS = {
    # Mobile base only (3 DOF, holonomic; pan and tilt held)
    "ridgeback_base": {"dof": 3, "joints": _BASE_JOINTS},
    # Base + pan-tilt unit (5 DOF, full)
    "ridgeback_whole_body": {"dof": 5, "joints": _BASE_JOINTS + _CAMERA_JOINTS},
}

# Base at the origin, camera looking straight ahead.
HOME_JOINTS = np.zeros(5)
