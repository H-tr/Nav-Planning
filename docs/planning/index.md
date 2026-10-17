# Motion Planning

<div class="grid cards" markdown>

-   __OMPL frontend__

    ---

    All of [OMPL](https://ompl.kavrakilab.org/)'s mature sampling-based
    planners (31 geometric, 4 multilevel), driven by one uniform Python
    call.  No glue code to swap planners — change a string.

-   __VAMP backend__

    ---

    Collision checking runs through [VAMP](https://github.com/KavrakiLab/vamp)'s
    SIMD-vectorised sphere pipeline against point-cloud obstacles.

-   __C++ hot path__

    ---

    Python calls `plan(start, goal)` once.  Everything inside —
    sampling, NN queries, collision checks, path simplification —
    runs in C++ behind a single nanobind call.

</div>

## Architecture

```
         Python
           │  planner.plan(start, goal)
           ▼
    ┌─────────────────────────────────────────┐
    │     OMPL planner + ProblemDefinition    │
    │  ┌────────────┐  ┌──────────────────┐   │
    │  │  Planner   │→ │ MotionValidator  │   │
    │  │  RRT-C,    │  │  samples the     │   │
    │  │  BIT*, …   │  │  base curve      │   │
    │  └────────────┘  └────────┬─────────┘   │
    │                           ▼             │
    │                  ┌─────────────────┐    │
    │                  │ VAMP SIMD       │    │
    │                  │  sphere-cloud   │    │
    │                  │  collision      │    │
    │                  └─────────────────┘    │
    └─────────────────────────────────────────┘
```

The frontend is vanilla OMPL.  The collision checker is the only
replaced component: it swaps OMPL's per-state callback for a VAMP
pipeline that batches many sphere/point checks into one AVX2 kernel.

The base `(x, y, theta)` is planned in SE(2), composed with a
RealVectorStateSpace for the camera joints:

- **LoCoBot** — [`ReedsSheppStateSpace`](https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html):
  a differential drive moves forward or backward along its heading and
  turns; edges are Reeds-Shepp curves with a 0.242 m turning radius.
- **Ridgeback** — [`SE2StateSpace`](https://ompl.kavrakilab.org/classompl_1_1base_1_1SE2StateSpace.html):
  a mecanum base moves in x, y and theta independently; edges are
  straight in x and y with the shortest-arc heading.

The motion validator samples each edge along that curve at VAMP's
resolution, so path simplification and interpolation, which use the same
curves, stay collision-free.

## Supported planners

The `planner_name` field of
[`PlannerConfig`](../api/types.md#nav_planning.types.planning.PlannerConfig)
accepts any of:

=== "Feasibility (single-query, returns on first solution)"

    | Family | Planners |
    |---|---|
    | RRT | `rrtc`, `rrt`, `trrt`, `bitrrt`, `strrtstar`, `lbtrrt` |
    | KPIECE | `kpiece`, `bkpiece`, `lbkpiece` |
    | PRM | `prm`, `lazyprm`, `spars`, `spars2` |
    | Exploration | `est`, `biest`, `sbl`, `stride`, `pdst` |
    | RRT\*-feasible | `rrtsharp`, `rrtxstatic` |
    | Multilevel | `qrrt`, `qmp` |

=== "Asymptotically optimal (keep refining until budget)"

    | Family | Planners |
    |---|---|
    | RRT\* | `rrtstar`, `informed_rrtstar` |
    | Informed trees | `bitstar`, `abitstar`, `aitstar`, `eitstar`, `blitstar` |
    | FMT | `fmt`, `bfmt` |
    | PRM\* | `prmstar`, `lazyprmstar` |
    | Multilevel | `qrrtstar`, `qmpstar` |

Every planner returns on the first exact solution.  The multilevel
planners (OMPL fiber bundles) plan the base first, in SE(2) against the
links that do not move with the camera, then lift the path into the
whole body.

## Minimal example

```python
import numpy as np

from nav_planning.planning import create_planner
from nav_planning.types import PlannerConfig

planner = create_planner(
    "ridgeback_whole_body",
    config=PlannerConfig(planner_name="rrtc", time_limit=1.0),
    pointcloud=obstacle_cloud,           # (N, 3) np.float32
)
planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)

start = np.zeros(5)                      # x, y, theta, pan, tilt
goal = np.array([5.5, 1.9, np.pi / 2, 1.5, -0.5])
result = planner.plan(start, goal)
print(result.success, result.planning_time_ns * 1e-6, "ms")
```

## Configuring a plan

Every plan is driven by a single
[`PlannerConfig`](../api/types.md#nav_planning.types.planning.PlannerConfig)
dataclass:

```python
PlannerConfig(
    # Which OMPL planner to run — see "Supported planners" above.
    planner_name="rrtc",

    # Wall-clock ceiling for the search.
    time_limit=1.0,

    # Radius of every obstacle point.  The robot's spheres already enclose
    # its meshes, so the default is 0.
    point_radius=0.0,

    # After the raw path, run OMPL's shortcutter.
    simplify=True,

    # Resample the (simplified) path densely along the base curves.
    #   interpolate_count > 0   → exactly this many total waypoints
    #   resolution > 0.0        → ceil(edge_length * resolution) samples
    #                             per edge
    #   both 0                  → OMPL's default longest-valid-segment
    #                             fraction
    interpolate=True,
    resolution=64.0,
    interpolate_count=0,
)
```

With `interpolate=False` consecutive waypoints are joined by the base's
own curve, not by a straight line in joint space:
[`interpolate_path`](../api/planning.md#nav_planning.planning.motion_planner.MotionPlanner.interpolate_path)
samples it later.

## Standalone collision checking

```python
planner.validate(cfg)                   # single config → bool
planner.validate_batch(cfgs)            # (N, ndof) → (N,) bool array
```

`validate_batch` packs up to `rake` distinct configurations into one
VAMP `ConfigurationBlock<rake>` and runs one `fkcc<rake>` call per
block — the primitive the motion validator uses for the samples of an
edge.

## More

<div class="grid cards" markdown>

-   [__Subgroup planning__](subgroup.md)

    ---

    The base alone with the camera held, or the whole body, and the
    low-level API for custom subgroups.

-   [__Time parameterization__](trajectory.md)

    ---

    Convert geometric paths into time-optimal trajectories with
    per-joint velocity and acceleration limits.

-   [__Kinodynamic planning__](kinodynamic.md)

    ---

    FLASK: plan a time-parameterised trajectory directly, within the
    joint and base limits.

</div>
