# Kinodynamic Planning (FLASK)

`MotionPlanner.plan_kinodynamic` plans a **time-parameterised,
dynamically feasible trajectory** directly — no separate time
parameterization step.  It implements FLASK (Duong et al., "Ultrafast
Sampling-based Kinodynamic Planning via Differential Flatness", T-RO
2026): RRT-Connect in the differentially flat output space, where every
tree edge is a closed-form cubic that is collision-checked with the same
SIMD VAMP kernels as the geometric planner.

```python
from nav_planning.planning import create_planner
from nav_planning.types import KinodynamicConfig

planner = create_planner("locobot_whole_body", pointcloud=cloud)
result = planner.plan_kinodynamic(start, goal, config=KinodynamicConfig(time_limit=1.0))

traj = result.trajectory                      # KinodynamicTrajectory
times, q, qd, qdd = traj.sample_uniform(dt=0.01)
twist = traj.sample_base_twist(times)         # (T, 3): v_x, v_y, omega
```

It works for every subgroup of both robots.

## How it works

<div class="grid cards" markdown>

-   __Flat outputs__

    ---

    The camera joints are their own flat outputs (fully actuated).  The
    **Ridgeback**'s holonomic base contributes x, y and theta (periodic:
    edges turn the short way).  The **LoCoBot**'s differential drive
    contributes its planar position `(x, y)`; the heading follows the
    direction of travel, `theta = atan2(y_dot, x_dot) + gear * pi`.

-   __Closed-form edges__

    ---

    Nodes are `(position, velocity)` states.  An edge is the cubic
    Hermite between its endpoints — the minimum-effort (LQMT) motion —
    with the duration `T*` that minimises
    `sum_i int y_i''^2 / a_max_i^2 dt + rho T` (a quartic root).

-   __Limits__

    ---

    Joint position / velocity / acceleration bounds — and the
    Ridgeback's yaw rate and yaw acceleration — are checked exactly on
    each cubic; `T*` is stretched until they hold.  The base's
    body-frame velocity and acceleration limits are checked on a 50 ms
    grid: forward speed, tangential acceleration and yaw rate for the
    LoCoBot; forward and lateral speed and their rates of change for the
    Ridgeback, the quantities its velocity controller limits.

-   __SIMD collision checks__

    ---

    Each edge is sampled at VAMP's resolution along its arc length and
    checked in interleaved `ConfigurationBlock` batches (FLASK Alg. 3),
    then the solution is simplified the way VAMP simplifies geometric
    paths: greedy shortcuts (Alg. 4) plus subdivide-and-cut-corners
    smoothing, keeping a change only if it lowers the cost.

</div>

### The LoCoBot at rest

The unicycle model has no heading at zero velocity — exactly where a
mobile robot starts and stops.  The planner makes parked states
first-class:

* A **parked node** keeps its heading.  Edges leave and enter it along
  that heading (forward or reverse); when a tree grows out of a parked
  node, the sampled target's lateral velocity is projected so the cubic
  departs aligned.
* **Gears** only change at parked nodes, so every drive segment has a
  well-defined heading and no hidden cusp.
* **Rotate in place**: a connection into or out of a parked node whose
  heading does not match gets a spin segment (camera holding still).
  Between two parked nodes that is rotate–translate–rotate, which is why
  open-floor legs solve in under a millisecond.

The output is exactly nonholonomic: the lateral velocity is zero.  The
LoCoBot's yaw acceleration is bounded on spin segments; while driving it
follows from the path's curvature.

## Tracking the trajectory

The trajectory is C¹ in every flat output: positions and velocities are
continuous; accelerations — and the LoCoBot's yaw rate, which depends on
them — jump at segment knots.  A feed-forward + feedback controller is
enough:

* **Joints**: `u = qd_ref + Kp (q_ref - q) + Ki ∫(q_ref - q)` as a
  velocity command.
* **Base**: feed-forward `(v_x, v_y, omega)` from `traj.base_twist(t)`
  plus pose feedback in the body frame.

Pass the current state to replan on the fly — the new trajectory starts
with the given velocity:

```python
q_now, qd_now = traj.position(t), traj.velocity(t)
result = planner.plan_kinodynamic(q_now, new_goal, start_velocity=qd_now)
```

Trajectories are collision-free at zero clearance, up to the sampling
resolution, like every VAMP edge: re-checked every 2 ms, 2 of the 240
demo-scene trajectories touch an obstacle point between two collision
samples ([results](#results-on-the-demo-scene)).

## Configuration

`KinodynamicConfig` holds the planner parameters; the limits come from
`nav_planning/locobot.py` and `nav_planning/ridgeback.py`
(`JOINT_VELOCITY_LIMITS`, `JOINT_ACCELERATION_LIMITS`, `BASE_MAX_*`) and
can be scaled with `velocity_scale` / `acceleration_scale`.

| Field | Default | Meaning |
|---|---|---|
| `time_limit` | `1.0` | Planning budget (s) |
| `rho` | `1.0` | Time weight; rest-to-rest edges peak at `sqrt(rho)` × the acceleration limit |
| `max_extension_time` | `3.0` | Tree step: longer cubics are truncated (node stays on the cubic) |
| `rest_sample_probability` | `0.3` | Share of zero-velocity samples (straight-line edges, gear changes) |
| `spin_probability` | `0.3` | Share of LoCoBot rotate-in-place extensions from parked nodes |
| `allow_reverse` | `True` | Permit the LoCoBot's reverse gear |
| `simplify_iterations` / `simplify_time_limit` | `5` / `0.05` | Rounds and time cap for the VAMP-style simplification |
| `seed` | `0` | RNG seed (`0` = nondeterministic) |

## Results on the demo scene

`examples/planning/kinodynamic.py` plans three legs of the demo scene
(examples/planning/scene.py: two rooms joined by a 1.2 m doorway, a
table, a low shelf, two pillars; 96k points) 20 times each, with FLASK
and with RRT-Connect (as PHANTOM calls it: `simplify=True,
interpolate=False`).  *Touch* counts the plans that touch an obstacle
point between two collision samples: FLASK trajectories re-checked
every 2 ms, RRT-Connect paths densified along the base curves 8× finer
than the motion checks.
Measured with `pixi run python examples/planning/kinodynamic.py` at
commit 93a43d4 on 2026-10-19, single-threaded on an AMD EPYC 7543; raw
output in `/nas/pollux/tianrun/nav_geo/demo_scene_93a43d4.txt`.

| Subgroup | Leg | FLASK success | FLASK median / p95 | Duration | FLASK touch | RRT-Connect success | RRT-Connect median / p95 | RRT-Connect touch |
|---|---|---|---|---|---|---|---|---|
| locobot_base | room 2 | 20/20 | 0.3 / 0.3 ms | 22.5 s | 0 | 20/20 | 2.2 / 7.2 ms | 0 |
| locobot_base | behind pillar | 20/20 | 51.7 / 105.8 ms | 17.3 s | 1 | 20/20 | 3.1 / 9.5 ms | 0 |
| locobot_base | table | 20/20 | 12.7 / 67.7 ms | 8.2 s | 0 | 20/20 | 1.0 / 1.7 ms | 0 |
| locobot_whole_body | room 2 | 20/20 | 0.4 / 0.4 ms | 22.5 s | 0 | 20/20 | 2.3 / 4.8 ms | 0 |
| locobot_whole_body | behind pillar | 20/20 | 52.4 / 93.1 ms | 16.7 s | 0 | 20/20 | 2.8 / 15.8 ms | 1 |
| locobot_whole_body | table | 20/20 | 10.9 / 57.4 ms | 6.6 s | 1 | 20/20 | 0.9 / 1.6 ms | 0 |
| ridgeback_base | room 2 | 20/20 | 0.6 / 0.7 ms | 8.2 s | 0 | 20/20 | 16.0 / 42.9 ms | 0 |
| ridgeback_base | behind pillar | 20/20 | 36.4 / 146.3 ms | 9.1 s | 0 | 20/20 | 8.5 / 41.6 ms | 1 |
| ridgeback_base | table | 20/20 | 0.2 / 0.2 ms | 3.2 s | 0 | 20/20 | 0.6 / 1.2 ms | 0 |
| ridgeback_whole_body | room 2 | 20/20 | 0.7 / 0.7 ms | 8.6 s | 0 | 20/20 | 7.5 / 61.3 ms | 0 |
| ridgeback_whole_body | behind pillar | 20/20 | 33.4 / 131.0 ms | 9.7 s | 0 | 20/20 | 24.0 / 66.4 ms | 1 |
| ridgeback_whole_body | table | 20/20 | 0.2 / 0.2 ms | 3.4 s | 0 | 20/20 | 0.7 / 1.7 ms | 0 |

Sampled every 2 ms, the largest velocity or acceleration of any of the
240 FLASK trajectories was 1.000 × its limit (base speed and yaw rate),
and the LoCoBot's lateral speed was exactly zero.  Open-floor legs plan
in under a millisecond (a direct connection); legs that steer around
the pillar take tens of milliseconds, including up to 50 ms
(`simplify_time_limit`) of simplification.
