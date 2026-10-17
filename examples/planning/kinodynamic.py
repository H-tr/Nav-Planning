"""Kinodynamic (FLASK) and geometric planning on the demo scene.

For every subgroup of both robots, plans each leg of the demo scene
(examples/planning/scene.py) ``TRIALS`` times with FLASK and with
RRT-Connect, and prints the success counts, the median and 95th
percentile planning times, and the trajectory durations.  Every FLASK
trajectory is sampled every 2 ms and checked against the robot's
limits; the largest ratio of a sampled velocity / acceleration to its
limit is printed (1.00 = at the limit), with the LoCoBot's largest
lateral speed.

    pixi run python examples/planning/kinodynamic.py
"""

import numpy as np
import scene

from nav_planning import locobot, ridgeback
from nav_planning.planning import create_planner
from nav_planning.types import KinodynamicConfig, PlannerConfig

TRIALS = 20
DT = 0.002
CAMERA_GOAL = {"locobot": (0.8, 0.4), "ridgeback": (1.5, -0.5)}


def limit_ratios(robot, planner, traj):
    """Largest sampled ratio of each limited quantity to its limit."""
    t, q, qd, qdd = traj.sample_uniform(DT)
    twist = traj.sample_base_twist(t)
    ratios = {
        "base speed": np.abs(twist[:, :2]).max() / robot.BASE_MAX_SPEED,
        "yaw rate": np.abs(twist[:, 2]).max() / robot.BASE_MAX_YAW_RATE,
    }
    # Body-frame acceleration (the LoCoBot's is tangential).
    dv = np.gradient(twist[:, :2], t, axis=0)[2:-2]
    ratios["base accel"] = np.abs(dv).max() / robot.BASE_MAX_ACCELERATION
    for i, name in enumerate(planner.joint_names[3:], start=3):
        ratios[f"{name} vel"] = (
            np.abs(qd[:, i]).max() / robot.JOINT_VELOCITY_LIMITS[name]
        )
        ratios[f"{name} acc"] = (
            np.abs(qdd[:, i]).max() / robot.JOINT_ACCELERATION_LIMITS[name]
        )
    lateral = np.abs(twist[:, 1]).max()
    return ratios, lateral


def main():
    cloud = scene.room_pointcloud()
    worst, lateral = {}, 0.0
    print(
        "| Subgroup | Leg | FLASK success | FLASK median / p95 | Duration "
        "| RRT-Connect success | RRT-Connect median / p95 |"
    )
    print("|---|---|---|---|---|---|---|")
    for robot in (locobot, ridgeback):
        for subgroup in robot.PLANNING_SUBGROUPS:
            planner = create_planner(
                subgroup,
                config=PlannerConfig(
                    planner_name="rrtc", time_limit=2.0, interpolate=False
                ),
                pointcloud=cloud,
            )
            planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)
            for leg, goal in scene.GOALS.items():
                start, goal = np.array(scene.START), np.array(goal)
                if subgroup.endswith("whole_body"):
                    start = np.r_[start, 0.0, 0.0]
                    goal = np.r_[goal, CAMERA_GOAL[robot.ROBOT_NAME]]
                flask, rrtc, durations = [], [], []
                for trial in range(TRIALS):
                    cfg = KinodynamicConfig(time_limit=2.0, seed=trial + 1)
                    k = planner.plan_kinodynamic(start, goal, config=cfg)
                    if k.success:
                        flask.append(k.planning_time_ns * 1e-6)
                        durations.append(k.trajectory.duration)
                        ratios, lat = limit_ratios(robot, planner, k.trajectory)
                        for key, value in ratios.items():
                            key = f"{robot.ROBOT_NAME} {key}"
                            worst[key] = max(worst.get(key, 0.0), value)
                        if robot is locobot:
                            lateral = max(lateral, lat)
                    g = planner.plan(start, goal)
                    if g.success:
                        rrtc.append(g.planning_time_ns * 1e-6)
                print(
                    f"| {subgroup} | {leg} | {len(flask)}/{TRIALS} "
                    f"| {np.median(flask):.1f} / {np.percentile(flask, 95):.1f} ms "
                    f"| {np.median(durations):.1f} s "
                    f"| {len(rrtc)}/{TRIALS} "
                    f"| {np.median(rrtc):.1f} / {np.percentile(rrtc, 95):.1f} ms |"
                )
    print("\nLargest sampled value / limit over all FLASK trajectories:")
    for key, value in worst.items():
        print(f"  {key:28s} {value:.3f}")
    print(f"  locobot lateral speed        {lateral:.2e} m/s")


if __name__ == "__main__":
    main()
