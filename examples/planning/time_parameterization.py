"""Time-parameterize a geometric Ridgeback path with TOPP-RA.

Plans the Ridgeback's whole body (holonomic base + pan-tilt unit) across
the demo scene with RRT-Connect, then computes the time-optimal
trajectory under the per-axis limits of nav_planning/ridgeback.py.  The
heading is unwrapped first: the base turns the short way round, which
the wrapped angles of the path do not say.

For the LoCoBot's differential drive use plan_kinodynamic: TOPP-RA
times each joint independently and does not keep the base on its
nonholonomic curves.

    pixi run python examples/planning/time_parameterization.py
"""

import numpy as np
import scene

from nav_planning import ridgeback
from nav_planning.planning import create_planner
from nav_planning.trajectory import TimeOptimalParameterizer
from nav_planning.types import PlannerConfig

planner = create_planner(
    "ridgeback_whole_body",
    config=PlannerConfig(planner_name="rrtc", time_limit=2.0, interpolate=False),
    pointcloud=scene.room_pointcloud(),
)
planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)
start = np.r_[scene.START, 0.0, 0.0]
goal = np.r_[scene.GOALS["behind pillar"], 1.5, -0.5]
result = planner.plan(start, goal)
path = result.path.copy()
path[:, 2] = np.unwrap(path[:, 2])

camera = planner.joint_names[3:]
vel = [ridgeback.BASE_MAX_SPEED] * 2 + [ridgeback.BASE_MAX_YAW_RATE]
vel += [ridgeback.JOINT_VELOCITY_LIMITS[j] for j in camera]
acc = [ridgeback.BASE_MAX_ACCELERATION] * 2 + [ridgeback.BASE_MAX_YAW_ACCELERATION]
acc += [ridgeback.JOINT_ACCELERATION_LIMITS[j] for j in camera]
traj = TimeOptimalParameterizer(vel, acc).parameterize(path)

times, q, qd, qdd = traj.sample_uniform(dt=0.01)
print(f"{len(path)} waypoints -> {traj.duration:.2f} s, {len(times)} samples at 100 Hz")
print(f"peak |qd|  = {np.round(np.abs(qd).max(axis=0), 3)}")
print(f"peak |qdd| = {np.round(np.abs(qdd).max(axis=0), 3)}")
