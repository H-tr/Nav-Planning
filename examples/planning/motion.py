"""Geometric planning on the demo scene, shown in the PyBullet GUI.

Plans the base of the robot from the start of the demo scene to each
goal with RRT-Connect (pan and tilt held), densifies the path along the
base's own curves, and plays it back.

    pixi run -e dev python examples/planning/motion.py locobot_base
    pixi run -e dev python examples/planning/motion.py ridgeback_whole_body
"""

import sys

import scene

from nav_planning import locobot, ridgeback
from nav_planning.envs.pybullet_env import PyBulletEnv
from nav_planning.planning import create_planner
from nav_planning.types import PlannerConfig


def main(subgroup: str = "locobot_base") -> None:
    robot = locobot if subgroup.startswith("locobot") else ridgeback
    cloud = scene.room_pointcloud()
    planner = create_planner(
        subgroup,
        config=PlannerConfig(planner_name="rrtc", time_limit=2.0),
        pointcloud=cloud,
    )
    planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)

    env = PyBulletEnv(getattr(robot, f"{robot.ROBOT_NAME}_robot_config"))
    env.add_pointcloud(cloud)
    start = planner.extract_config(robot.HOME_JOINTS)
    for name, base in scene.GOALS.items():
        goal = start.copy()
        goal[:3] = base
        result = planner.plan(start, goal)
        print(
            f"{name}: {result.status.value} in {result.planning_time_ns * 1e-6:.1f} ms"
        )
        if result.success:
            env.animate_path(planner.embed_path(result.path), next_key="n")


if __name__ == "__main__":
    main(*sys.argv[1:])
