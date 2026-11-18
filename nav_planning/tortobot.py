"""TortoBot's bundled description.

This module collects every concrete value that describes TortoBot:
joint groupings, the home pose, the planning subgroups, the kinodynamic
limits and the top-level ``RobotConfig`` instance.

TortoBot is the planar robot of PHANTOM (github.com/H-tr/PHANTOM-Planner,
``phantom/robot/tortobot.urdf``): a disc base of radius 0.32 m whose x, y
and heading move on their own, and a camera that pans on its own joint.
Its collision model is one sphere of the disc's radius at the disc's
centre.  It is a model for planar maps: give the map as an ESDF whose
value does not change with height (the same layer repeated, with the
lattice reaching at least 0.32 m above and below z = 0).  The sphere is
then free exactly where the disc is free.

The full configuration vector is 4-DOF:

* ``[0:3]`` — mobile base ``(x, y, theta)``, holonomic
* ``[3:4]`` — ``camera_pan``

The dataclass *types* live in :mod:`nav_planning.types.robot`; this file
holds *values* of those types.
"""

import os

import numpy as np

from nav_planning.types.robot import CameraConfig, RobotConfig

_PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
_RESOURCES_DIR = os.path.join(_PKG_ROOT, "resources", "robot", "tortobot")

# The C++ planner's name for this robot.
ROBOT_NAME = "tortobot"

# Atomic joint groups — indices into the full 4-DOF configuration array.
# Order matches VAMP's URDF tree traversal: base → pan.
JOINT_GROUPS = {
    "base": slice(0, 3),  # base_x, base_y, base_theta (holonomic)
    "camera": slice(3, 4),  # camera_pan
}

# Kinodynamic limits used by ``MotionPlanner.plan_kinodynamic`` (FLASK):
# the simulated actuator limits of PHANTOM's TortoBot description.  PHANTOM
# gives 1.0 m/s and 0.5 m/s^2 for each world axis; FLASK takes them as its
# limits for each body axis.
JOINT_VELOCITY_LIMITS: dict[str, float] = {"camera_pan": 3.0}  # rad/s
JOINT_ACCELERATION_LIMITS: dict[str, float] = {"camera_pan": 300.0}  # rad/s^2
BASE_MAX_SPEED: float = 1.0  # m/s, each axis
BASE_MAX_ACCELERATION: float = 0.5  # m/s^2, each axis
BASE_MAX_YAW_RATE: float = 1.0  # rad/s
BASE_MAX_YAW_ACCELERATION: float = 2.0  # rad/s^2

VIZ_URDF_PATH = os.path.join(_RESOURCES_DIR, "tortobot.urdf")

tortobot_robot_config = RobotConfig(
    urdf_path=os.path.join(_RESOURCES_DIR, "tortobot.urdf"),
    joint_names=[
        # [0:3]   mobile base (virtual planar joints)
        "base_x_joint",
        "base_y_joint",
        "base_theta_joint",
        # [3:4]   camera
        "camera_pan",
    ],
    # PHANTOM's camera: 62 deg (1.0821 rad) horizontal field of view and a
    # 4.5 m range; a 4:3 image then has a 48.5 deg vertical field of view.
    camera=CameraConfig(
        link_name="camera_link",
        width=640,
        height=480,
        fov=48.5,
        near=0.1,
        far=4.5,
    ),
)

_BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_theta_joint"]
_CAMERA_JOINTS = ["camera_pan"]

PLANNING_SUBGROUPS = {
    # Mobile base only (3 DOF, holonomic; the camera held)
    "tortobot_base": {"dof": 3, "joints": _BASE_JOINTS},
    # Base + camera pan (4 DOF, full)
    "tortobot_whole_body": {"dof": 4, "joints": _BASE_JOINTS + _CAMERA_JOINTS},
}

# Base at the origin, camera looking straight ahead.
HOME_JOINTS = np.zeros(4)
