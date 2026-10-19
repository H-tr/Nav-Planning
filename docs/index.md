# Nav Planning

A planning library for two mobile robots with a pan-tilt camera — the
**Trossen LoCoBot** (differential drive) and the **Clearpath Ridgeback**
(holonomic) — with motion planning (OMPL frontend, VAMP SIMD collision
backend), kinodynamic planning (FLASK) and time-optimal trajectory
generation, all behind a unified Python API.  A sister library of
[Fetch-Planning](https://github.com/H-tr/Fetch-Planning) and
[Autolife-Planning](https://github.com/H-tr/Autolife-Planning).

<div class="grid cards" markdown>

-   :material-package-variant:{ .lg .middle } __Drop-in, few deps__

    ---

    For inference: **`numpy` and `pin`**.  No conda required, no ROS, no
    MoveIt.  `pip install nav-planning`.

-   :material-robot-mower-outline:{ .lg .middle } __Two robots, one API__

    ---

    `create_planner("locobot_base")`, `create_planner("ridgeback_whole_body")`
    — the same planners, trajectories and low-level calls for both
    robots; only the name changes.

-   :material-speedometer:{ .lg .middle } __Millisecond plans__

    ---

    VAMP's SIMD sphere checks against point clouds: on the two-room
    demo scene RRT-Connect plans in 0.6–24 ms and FLASK returns
    dynamically feasible trajectories in 0.2–52 ms (medians,
    [measurement](planning/kinodynamic.md#results-on-the-demo-scene)).

</div>

## Features

<div class="grid cards" markdown>

-   :material-map-marker-path:{ .lg .middle } [__Motion Planning__](planning/index.md)

    ---

    OMPL frontend + VAMP SIMD backend.  The LoCoBot's base is planned
    on Reeds-Shepp curves, the Ridgeback's in holonomic SE(2); the
    camera joints alongside, or held.

-   :material-timer-outline:{ .lg .middle } [__Time Parameterization__](planning/trajectory.md)

    ---

    Time-optimal path parameterization (TOPP-RA) with per-joint velocity
    and acceleration limits.

-   :material-run-fast:{ .lg .middle } [__Kinodynamic Planning__](planning/kinodynamic.md)

    ---

    FLASK plans a time-parameterised trajectory directly, within the
    joint and base limits — exactly nonholonomic for the LoCoBot.

-   :material-cube-scan:{ .lg .middle } [__Robots__](robots.md)

    ---

    Collision spheres generated from the official robot descriptions;
    a documented camera mast for the Ridgeback.

</div>

## Quick Install

For inference — running the planners — pip install the prebuilt wheel
(Linux x86_64, Python 3.10–3.14):

```bash
pip install nav-planning
```

See the [Getting Started](getting-started.md) guide for the full
development setup (pixi + conda-forge toolchain, robot description
rebuilds, FK codegen).
