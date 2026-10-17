# Examples

The examples share the demo scene in `examples/planning/scene.py`: two
rooms joined by a 1.2 m doorway, a table (0.75 m top: the LoCoBot fits
under it, the Ridgeback's mast does not), a 0.45 m shelf (the LoCoBot's
camera tower does not fit under it) and two pillars, as a 96k-point
cloud without the floor.

---

## Motion Planning around Obstacles

`examples/planning/motion.py` plans the base of a robot to each goal of
the scene with RRT-Connect and plays the paths back in the PyBullet GUI
(press `n` for the next goal).

```bash
pixi run -e dev python examples/planning/motion.py locobot_base
pixi run -e dev python examples/planning/motion.py ridgeback_whole_body
```

```python
import scene
from nav_planning import locobot
from nav_planning.envs.pybullet_env import PyBulletEnv
from nav_planning.planning import create_planner
from nav_planning.types import PlannerConfig

cloud = scene.room_pointcloud()
planner = create_planner(
    "locobot_base",
    config=PlannerConfig(planner_name="rrtc", time_limit=2.0),
    pointcloud=cloud,
)
planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)

result = planner.plan(planner.extract_config(locobot.HOME_JOINTS), goal)
env = PyBulletEnv(locobot.locobot_robot_config)
env.add_pointcloud(cloud)
env.animate_path(planner.embed_path(result.path))
```

---

## Kinodynamic Planning

`examples/planning/kinodynamic.py` plans every leg of the scene for
every subgroup of both robots with FLASK and with RRT-Connect, and
prints success counts, planning times, trajectory durations and how
close the trajectories come to the robots' limits (see
[Kinodynamic planning](planning/kinodynamic.md#results-on-the-demo-scene)).

```bash
pixi run python examples/planning/kinodynamic.py
```

---

## Time Parameterization

`examples/planning/time_parameterization.py` plans a Ridgeback
whole-body path with RRT-Connect and times it with TOPP-RA under the
Ridgeback's limits.

```bash
pixi run python examples/planning/time_parameterization.py
```

```python
from nav_planning.trajectory import TimeOptimalParameterizer

path[:, 2] = np.unwrap(path[:, 2])        # the heading takes the short way
param = TimeOptimalParameterizer(vel_limits, acc_limits)
traj = param.parameterize(path)
times, positions, velocities, accelerations = traj.sample_uniform(dt=0.01)
```

---

## Reference

### Joint groups

Indices into the full 5-DOF configuration of either robot:

| Group | Indices | LoCoBot | Ridgeback |
|-------|---------|---------|-----------|
| `base` | 0–2 | `base_x_joint`, `base_y_joint`, `base_theta_joint` | same |
| `camera` | 3–4 | `pan`, `tilt` | `ptu_pan`, `ptu_tilt` |
