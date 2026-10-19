# Subgroup Planning

Each robot exposes 5 joints — a 3-DOF mobile base `(x, y, theta)` and
the camera pan and tilt.  A subgroup is an ordered list of indices into
that body: the planner reasons in the reduced space, and the collision
pipeline injects the frozen joints and checks the **whole** robot.

## Built-in aliases

| Alias | DOF | What it controls |
|---|---:|---|
| `locobot_base` | 3 | LoCoBot base (x, y, yaw), camera held |
| `locobot_whole_body` | 5 | LoCoBot base + pan + tilt |
| `locobot` | 5 | alias for `locobot_whole_body` |
| `ridgeback_base` | 3 | Ridgeback base (x, y, yaw), PTU held |
| `ridgeback_whole_body` | 5 | Ridgeback base + PTU pan + tilt |
| `ridgeback` | 5 | alias for `ridgeback_whole_body` |

They are dicts in `nav_planning.locobot.PLANNING_SUBGROUPS` and
`nav_planning.ridgeback.PLANNING_SUBGROUPS`.

## Using a built-in alias

```python
from nav_planning.planning import create_planner
from nav_planning.types import PlannerConfig

live_config = robot.get_joint_state()          # 5-D array from your system

planner = create_planner(
    "locobot_base",
    config=PlannerConfig(time_limit=1.0),
    base_config=live_config,                    # pan / tilt held here
    pointcloud=obstacle_cloud,
)

start = planner.extract_config(live_config)     # 5-D → 3-D
result = planner.plan(start, goal)
full_path = planner.embed_path(result.path)     # 3-D path → 5-D path
```

`base_config` defaults to `HOME_JOINTS` (all zero).  Pass the live joint
reading to pin the camera exactly where it is.

`set_joint_limits({joint name: (lower, upper)})` sets the ranges the
planner samples in, for any joint of the full body: for example
`{"base_x_joint": (-1.0, 7.0), "base_y_joint": (-3.0, 3.0)}` for the
map's extent (the model spans ±10 m).  Joints left out keep their
compiled range, `None` restores every compiled range, and an unknown
name or a bad range raises.  With the base active the heading is
periodic, so the `base_theta_joint` range is not used.  `bounds` gives
the active joints' ranges, `joint_limits` the full body's.
`set_subgroup(name)` switches between a robot's subgroups and keeps the
point cloud and the joint ranges.

## The low-level API

The C++ planner takes the robot at construction and is otherwise the
`OmplVampPlanner` of Autolife-Planning:

```python
import numpy as np
from nav_planning._ompl_vamp import OmplVampPlanner

planner = OmplVampPlanner("ridgeback")            # full body, 5 DOF
lower, upper = planner.default_joint_limits()     # full-body order
lower[:2], upper[:2] = (-1.0, -3.0), (7.0, 3.0)   # base x / y box
planner.set_joint_limits(lower, upper)

planner.set_subgroup([0, 1, 2], [0.0, 0.0, 0.0, 0.3, -0.2])  # base; PTU held
planner.add_pointcloud(cloud.astype(np.float32), 0.0)         # replaces the cloud
result = planner.plan(start, goal, planner_name="rrtc", time_limit=1.0,
                      simplify=True, interpolate=False)
```

`OmplVampPlanner(robot, active_indices, frozen_config)` builds a
subgroup planner directly.  The base joints (0, 1, 2) are planned
together, leading the subgroup — any other combination with base
joints raises, as do indices outside 0–4, repeated indices and a frozen
configuration without five entries.  Joint limits are kept for the full body, so they survive
`set_subgroup`; `base_theta_joint` is periodic and its limits are not
used.

## When it matters

The base subgroup is the right choice for navigation with the camera
fixed; the whole body when the camera has to end up looking somewhere,
or when its pose matters for clearance — the LoCoBot's camera on its
tower, the Ridgeback's PTU on its mast.
