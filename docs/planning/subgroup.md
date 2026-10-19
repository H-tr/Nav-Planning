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

## What a camera-only subgroup's edges guarantee

Without the base (for example `OmplVampPlanner(robot, [3, 4], frozen)`),
an edge is checked by VAMP on the straight line between its ends, at
`8 n` evenly spaced samples, `n = ceil(L / 8 * 32)` for an edge of length
`L` (32 samples per unit).  The path is collision-free at those samples
only.  Planners that keep the valid start of a blocked motion (the
KPIECE family, PDST, STRIDE) add the last valid sample as a waypoint;
that edge was checked at the samples of the longer motion that lie on
it, which a fresh check of the edge does not reproduce.  Every waypoint
is a checked state.

On the camera-subgroup test scene (one obstacle point the camera hits at
pan 0, tilt 0; pan from −1.5 to 1.5 rad; `simplify=False`), the paths
that touch the point between those samples, densified 64 per unit:

| Planner | LoCoBot | Ridgeback |
|---|---:|---:|
| `kpiece` | 6 / 2000 | 0 / 2000 |
| `bkpiece` | 2 / 2000 | 0 / 2000 |
| `lbkpiece` | 7 / 2000 | 0 / 2000 |
| `pdst` | 9 / 2000 | 0 / 2000 |
| `stride` | 11 / 2000 | 0 / 2000 |

Of the 35 touching edges, 28 are valid at their own samples; the other
7 fail those but are valid at every point of an even spacing `L / m`
with `32 L <= m <= 64 L`, the samples a longer motion they were cut from
can have had.  A fresh
check of every edge rejects 1 of 1000 LoCoBot paths for `pdst` and for
`stride` (an edge cut from a longer motion).  Measured at commit 5fb2f7b
on 2026-10-19, single-threaded on an AMD EPYC 7543, with

```bash
pixi run python /nas/pollux/tianrun/nav_geo/camera_subgroup_rate.py locobot stride 2000
pixi run python /nas/pollux/tianrun/nav_geo/camera_subgroup_fresh.py locobot stride 1000
```

for each robot and planner; raw output in
`/nas/pollux/tianrun/nav_geo/camera_subgroup_rate.txt` and
`camera_subgroup_fresh.txt`.

## When it matters

The base subgroup is the right choice for navigation with the camera
fixed; the whole body when the camera has to end up looking somewhere,
or when its pose matters for clearance — the LoCoBot's camera on its
tower, the Ridgeback's PTU on its mast.
