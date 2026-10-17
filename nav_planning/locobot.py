"""The LoCoBot's bundled description.

This module collects every concrete value that describes the LoCoBot:
joint groupings, the home pose, the planning subgroups, the kinodynamic
limits and the top-level ``RobotConfig`` instance.

The robot is the arm-less ``locobot_base`` variant of the Trossen
Robotics / Interbotix LoCoBot on the iRobot Create 3 base, with the
pan-tilt Intel RealSense D435 camera tower and without the optional
lidar.  The description is the official ``interbotix_xslocobot_descriptions``
(github.com/Interbotix/interbotix_ros_rovers, ROS 2 Humble branch),
expanded by ``scripts/build_robot.sh``.

The full configuration vector is 5-DOF:

* ``[0:3]`` — mobile base ``(x, y, theta)``, nonholonomic differential drive
* ``[3:5]`` — camera ``pan`` and ``tilt``

The dataclass *types* live in :mod:`nav_planning.types.robot`; this file
holds *values* of those types.
"""

import math
import os

import numpy as np

from nav_planning.types.robot import CameraConfig, RobotConfig

_PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
_RESOURCES_DIR = os.path.join(_PKG_ROOT, "resources", "robot", "locobot")

# The C++ planner's name for this robot.
ROBOT_NAME = "locobot"

# Atomic joint groups — indices into the full 5-DOF configuration array.
# Order matches VAMP's URDF tree traversal: base → pan → tilt.
JOINT_GROUPS = {
    "base": slice(0, 3),  # base_x, base_y, base_theta (nonholonomic)
    "camera": slice(3, 5),  # pan, tilt
}

# Kinodynamic limits used by ``MotionPlanner.plan_kinodynamic`` (FLASK).
#
# Pan and tilt are the two axes of a Dynamixel 2XL430-W250
# (docs.trossenrobotics.com/interbotix_xslocobots_docs/specifications/
# locobot_base.html).  Interbotix's driver runs them with a velocity-based
# profile, ``profile_velocity: 131`` and ``profile_acceleration: 15``
# (interbotix_xslocobot_control/config/modes_base.yaml).  In the ROBOTIS
# e-manual units (0.229 rev/min and 214.577 rev/min^2) that is pi rad/s —
# the URDF's joint velocity limit — and 5.62 rad/s^2.
_PROFILE_ACCELERATION = 15 * 214.577 * 2.0 * math.pi / 3600.0  # rad/s^2
JOINT_VELOCITY_LIMITS: dict[str, float] = {
    "pan": math.pi,
    "tilt": math.pi,
}
JOINT_ACCELERATION_LIMITS: dict[str, float] = {
    "pan": _PROFILE_ACCELERATION,
    "tilt": _PROFILE_ACCELERATION,
}

# iRobot Create 3 (iroboteducation.github.io/create3_docs):
# * speed — 0.306 m/s with the safety features on, "its true maximum
#   speed of 0.460 m/s" with ``safety_override: full``, which Trossen's
#   LoCoBot configuration sets (Create 3 configuration page of the LoCoBot
#   docs);
# * acceleration — ``wheel_accel_limit`` defaults to its maximum,
#   900 mm/s^2 (ROS 2 parameters page);
# * yaw rate — iRobot gives no limit in the docs; its action interfaces
#   (irobot_create_msgs RotateAngle.action) and simulator controller
#   (create3_sim irobot_create_control/config/control.yaml) cap it at
#   1.9 rad/s;
# * yaw acceleration — no published value; create3_sim uses 7.725 rad/s^2,
#   two wheels at 0.9 m/s^2 in opposite directions on the 0.233 m wheel
#   separation.
BASE_MAX_SPEED: float = 0.46  # m/s
BASE_MAX_ACCELERATION: float = 0.9  # m/s^2
BASE_MAX_YAW_RATE: float = 1.9  # rad/s
BASE_MAX_YAW_ACCELERATION: float = 7.725  # rad/s^2

VIZ_URDF_PATH = os.path.join(_RESOURCES_DIR, "locobot.urdf")

locobot_robot_config = RobotConfig(
    urdf_path=os.path.join(_RESOURCES_DIR, "locobot.urdf"),
    joint_names=[
        # [0:3]   mobile base (virtual planar joints)
        "base_x_joint",
        "base_y_joint",
        "base_theta_joint",
        # [3:5]   camera
        "pan",
        "tilt",
    ],
    # RealSense D435 colour stream, 69 x 42 deg field of view
    # (Intel RealSense D400 series datasheet).
    camera=CameraConfig(
        link_name="locobot/camera_depth_link",
        width=640,
        height=360,
        fov=42.0,
        near=0.1,
        far=10.0,
    ),
)

_BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_theta_joint"]
_CAMERA_JOINTS = ["pan", "tilt"]

PLANNING_SUBGROUPS = {
    # Mobile base only (3 DOF, nonholonomic; pan and tilt held)
    "locobot_base": {"dof": 3, "joints": _BASE_JOINTS},
    # Base + camera (5 DOF, full)
    "locobot_whole_body": {"dof": 5, "joints": _BASE_JOINTS + _CAMERA_JOINTS},
}

# Base at the origin, camera looking straight ahead.
HOME_JOINTS = np.zeros(5)
