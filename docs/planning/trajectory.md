# Time Parameterization

The missing step between a geometric path and a command stream that a
motor controller can execute. The motion planner gives you **where** to
go — a list of waypoints in joint space — but not **when**. Time
parameterization adds the timing: it assigns a time stamp to every
point on the path so that per-joint velocity and acceleration limits
are respected, and the trajectory is as fast as physically possible.

<div class="grid cards" markdown>

-   __Time-optimal__

    ---

    Finds the fastest feasible velocity profile along the path.
    At every instant some joint is at its velocity or acceleration
    limit — there is no slack left to speed up.

-   __Bounded velocity + acceleration__

    ---

    Per-joint velocity and acceleration limits are hard constraints,
    enforced on a fine grid along the path (samples between grid
    points stay within a fraction of a percent of the limits).

-   __C++ hot path__

    ---

    The vendored TOPP-RA core is Eigen-only C++, exposed through a
    single nanobind call. Python overhead is one round-trip per path.

</div>

## Algorithm

The implementation is **TOPP-RA** (Time-Optimal Path Parameterization
based on Reachability Analysis) by Pham and Pham (2018). The C++ core
of [toppra](https://github.com/hungpham2511/toppra) (MIT) is vendored
under `ext/time_parameterization/toppra/` — joint velocity and
acceleration constraints, its built-in Seidel LP solver, piecewise
polynomial paths, and the constant-acceleration parametrizer.

The pipeline has three stages:

1. **Path spline** — the piecewise-linear planner path is resampled
   every `knot_spacing` along each segment and joined by a natural
   cubic spline in arc length. The spline passes through every
   waypoint, rounds the corners, and stays within about
   `knot_spacing / 10` of the original segments.
2. **Reachability analysis** — a backward pass over a grid along the
   path computes, at each grid point, the set of path velocities from
   which the end can still be reached; a forward pass then picks the
   largest admissible velocity at each point. Each step is a tiny
   linear program over the joint limits.
3. **Timing** — the path velocity profile is integrated with
   constant path acceleration between grid points, giving $q(t)$ with
   continuous velocity.

The result starts and ends at rest.

!!! note "Skip OMPL's interpolation step"

    Pass `interpolate=False` to `plan()` or `PlannerConfig` when you
    time-parameterize the output.  The parameterizer resamples the path
    itself; OMPL's dense interpolation only adds knots (and cost —
    the spline fit is cubic in the number of knots).

## Minimal example

```python
import numpy as np
from nav_planning import ridgeback
from nav_planning.planning import create_planner
from nav_planning.trajectory import TimeOptimalParameterizer
from nav_planning.types import PlannerConfig

# 1. Plan a collision-free path (skip interpolation).
planner = create_planner(
    "ridgeback_whole_body",
    config=PlannerConfig(simplify=True, interpolate=False),
    pointcloud=cloud,
)
result = planner.plan(start, goal)
path = result.path.copy()                 # (N, 5)

# 2. The heading is periodic: unwrap it so each edge turns the short way.
path[:, 2] = np.unwrap(path[:, 2])

# 3. Time-parameterize.
camera = planner.joint_names[3:]
vel = [ridgeback.BASE_MAX_SPEED] * 2 + [ridgeback.BASE_MAX_YAW_RATE]
vel += [ridgeback.JOINT_VELOCITY_LIMITS[j] for j in camera]
acc = [ridgeback.BASE_MAX_ACCELERATION] * 2 + [ridgeback.BASE_MAX_YAW_ACCELERATION]
acc += [ridgeback.JOINT_ACCELERATION_LIMITS[j] for j in camera]

param = TimeOptimalParameterizer(vel, acc)
traj  = param.parameterize(path)
print(f"Duration: {traj.duration:.3f} s")

# 4. Sample at controller rate.
times, positions, velocities, accelerations = traj.sample_uniform(dt=0.01)
```

!!! note "Bases"

    TOPP-RA bounds every joint independently.  For the Ridgeback the
    world-frame x / y bounds above are the controller's per-body-axis
    limits; along a straight base edge that keeps the body-frame speed
    within the limit only when the edge is aligned with an axis, so use
    them as a nominal timing (or scale them by 1/√2 for a strict bound).
    For the LoCoBot use [kinodynamic planning](kinodynamic.md): TOPP-RA
    rounds the corners of the path with a spline and does not keep a
    differential drive on its curves.

## Configuration knobs

| Parameter | Default | Description |
|---|---|---|
| `max_velocity` | *(required)* | `(ndof,)` per-joint velocity limit (rad/s or m/s) |
| `max_acceleration` | *(required)* | `(ndof,)` per-joint acceleration limit |
| `knot_spacing` | `0.1` | Spline knot spacing along the path (path units). The trajectory deviates from the straight segments by about a tenth of it; smaller follows the corners more tightly but corners more slowly. |
| `velocity_scaling` | `1.0` | Scale factor in `(0, 1]` applied to `max_velocity`. Use to slow the trajectory without changing the stored limits. |
| `acceleration_scaling` | `1.0` | Scale factor in `(0, 1]` applied to `max_acceleration`. |

## Querying the trajectory

The returned `Trajectory` object supports both point and batch queries:

```python
# Point queries at arbitrary time t.
pos = traj.position(t)            # (ndof,)
vel = traj.velocity(t)            # (ndof,)
acc = traj.acceleration(t)        # (ndof,)

# Batch: user-supplied time grid.
pos, vel, acc = traj.sample(times)            # each (T, ndof)

# Batch: uniform grid at controller dt — always includes t=0 and t=duration.
times, pos, vel, acc = traj.sample_uniform(dt=0.01)
```

## Scaling for slower motion

Pass `velocity_scaling` or `acceleration_scaling` to `parameterize()`
to slow the trajectory without reconstructing the parameterizer:

```python
traj_slow = param.parameterize(path, velocity_scaling=0.5)
# traj_slow.duration > traj.duration
```

## One-shot convenience

For scripts where you only parameterize a single path:

```python
from nav_planning.trajectory import parameterize_path

traj = parameterize_path(path, vel_limits, acc_limits)
```

## Pipeline recipe

A typical end-to-end pipeline:

```
plan(start, goal, simplify=True, interpolate=False)
        │
        ▼
  (N, ndof) path          geometric, no timing
        │
        ▼
  TimeOptimalParameterizer.parameterize(path)   spline + TOPP-RA
        │
        ▼
  Trajectory               q(t), q̇(t), q̈(t) with bounded vel/acc
        │
        ▼
  traj.sample_uniform(dt)  dense rollout at controller rate
        │
        ▼
  stream to hardware        (times, positions, velocities, accelerations)
```

## API reference

See the full API docs at [Trajectory API](../api/trajectory.md).
