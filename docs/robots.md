# Robots

Both robots share one layout: three virtual planar joints
`base_x_joint`, `base_y_joint`, `base_theta_joint` (prismatic x,
prismatic y, revolute z, from a `world` link to the robot's root), then
the camera pan and tilt.  Every other joint of the official descriptions
(wheels, wheel drops, rocker) is fixed at zero for planning.

## LoCoBot

The arm-less `locobot_base` LoCoBot on the **iRobot Create 3** base with
the pan-tilt **Intel RealSense D435** tower, from the official
[`interbotix_xslocobot_descriptions`](https://github.com/Interbotix/interbotix_ros_rovers)
(ROS 2 Humble branch, which carries the Create 3 version), expanded with
`robot_model:=locobot_base arm_model:=mobile_base base_model:=create3
use_lidar:=false` — the lidar is an option that Interbotix's launch files
leave off.  The Create 3's `base_link` lies on the floor.

| Joint | Range | Velocity | Acceleration |
|---|---|---|---|
| `pan` | ±90° | π rad/s | 5.62 rad/s² |
| `tilt` | −90° … +75° | π rad/s | 5.62 rad/s² |

| Base | Limit | Source |
|---|---|---|
| speed | 0.46 m/s | Create 3 "true maximum speed" with `safety_override: full`, which Trossen's LoCoBot configuration sets (0.306 m/s with the safety features on) |
| acceleration | 0.9 m/s² | Create 3 `wheel_accel_limit` default (its maximum) |
| yaw rate | 1.9 rad/s | iRobot's action / simulator cap; the docs give no limit |
| yaw acceleration | 7.725 rad/s² | iRobot's simulator (`create3_sim`): both wheels at 0.9 m/s² in opposite directions; no published value |

The pan and tilt are the two axes of a Dynamixel 2XL430-W250, run by
Interbotix's driver with `profile_velocity: 131` and
`profile_acceleration: 15`; in the ROBOTIS e-manual units that is π rad/s
(the URDF's limit) and 5.62 rad/s².  Citations are in
`nav_planning/locobot.py`.

The geometric planner's Reeds-Shepp turning radius is 0.46 / 1.9 =
0.242 m: the radius on which the base turns at top speed and top yaw
rate.

## Ridgeback

The **Clearpath Ridgeback** from the official
[`ridgeback_description`](https://github.com/ridgeback/ridgeback)
(`melodic-devel`), without payload accessories.  Its `base_link` sits
0.0259 m above the floor (wheel radius 0.0759 m minus axle offset
0.0500 m in the description).

The Ridgeback has no head.  **Nav-Planning adds a camera mast**
(`scripts/ridgeback_camera_mast.urdf.xacro`, through the description's
`RIDGEBACK_URDF_EXTRAS` payload hook):

| Part | Description | Mount |
|---|---|---|
| mast | 40 × 40 mm aluminium extrusion, 0.70 m (a box; our addition) | centre of the payload deck (`mid_mount`, 0.306 m above the floor) |
| pan-tilt unit | FLIR PTU-5, official [`flir_ptu_description`](https://github.com/ros-drivers/flir_ptu) model `ptu5` | top of the mast, base 1.006 m above the floor |
| camera | Intel RealSense D435, official [`realsense2_description`](https://github.com/realsenseai/realsense-ros) | the PTU's payload mount (`ptu_mount_link`) |

| Joint | Range | Velocity | Acceleration |
|---|---|---|---|
| `ptu_pan` | ±168° | 120°/s | 120°/s² (assumed) |
| `ptu_tilt` | −90° … +30° | 50°/s | 50°/s² (assumed) |

The ranges are the PTU-5 user manual's default user limits (the
`flir_ptu` xacro's ±178° / +31…−81° match no FLIR document) and the
speeds its "Axis Speeds" (the older datasheet lists 150°/s pan).  FLIR
publishes no acceleration ("Acceleration/Deceleration Programmable"):
the values assume each axis reaches its top speed in one second — set
them to the accelerations programmed on your PTU.

| Base (per body axis) | Limit | Source |
|---|---|---|
| speed x, y | 1.1 m/s | `ridgeback_control/config/control.yaml`; data sheet "MAX SPEED 1.1 m/s" |
| acceleration x, y | 2.5 m/s² | `control.yaml` |
| yaw rate | 2.0 rad/s | `control.yaml` |
| yaw acceleration | 1.0 rad/s² | `control.yaml` |

## Collision models

`scripts/build_robot.sh` expands the official xacro, puts the robot on
the planar joints, copies the meshes (STL, decimated to 20k faces where
larger) and writes the SRDF; `scripts/spherize_robot.sh` spherizes the
URDF with foam (medial sphere trees, up to 32 spheres a mesh) and cuts
the base model out of it; `scripts/generate_fk.sh` runs cricket on both
models.

| Model | Spheres | Used by |
|---|---:|---|
| `LocobotWholeBody` | 312 | every LoCoBot check |
| `LocobotBase` | 228 | no planner |
| `RidgebackWholeBody` | 309 | every Ridgeback check |
| `RidgebackBase` | 214 | no planner |

Coverage (`scripts/sphere_coverage.py`: 20k surface samples per link
plus its vertices, tested against the link's spheres):

| Robot | Samples covered | Worst link | Deepest uncovered point |
|---|---:|---|---:|
| LoCoBot | 99.21 % | `camera_locobot_link` 95.8 % | 15.8 mm (`camera_tower_link`) |
| Ridgeback | 99.77 % | `riser_link` 99.4 % | 5.3 mm (`top_link`) |

The uncovered samples are where foam had to repair a mesh that is not
watertight (the camera tower, the camera) and where medial spheres round
off box edges.  The spheres also reach beyond the meshes: up to 5 cm
around the LoCoBot's tower and 13 cm around the Ridgeback's box-shaped
chassis, and up to 10 cm below the floor under the Ridgeback — **leave
the floor out of the point cloud**.

Self-collision exclusions (`<robot>.srdf`): link pairs on the same body
are never checked; a pair on different bodies is disabled when the
bodies are adjacent, when a pan × tilt survey of the meshes shows they
never touch (or always touch), or when their spheres overlap at the home
pose.  The LoCoBot's tilt bracket and camera tower are such a pair: the
meshes touch only at the corners of the joint range (pan ±90°, tilt
≥ 60°), but the spheres already overlap at home, so the pair is not
checked.  Neither robot's model has a checked self-collision pair.
