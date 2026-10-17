"""Public API: robot constants, the low-level planner, kinematics."""

import numpy as np
import pytest
import scene

from nav_planning import locobot, ridgeback
from nav_planning._ompl_vamp import OmplVampPlanner
from nav_planning.kinematics import compute_forward_kinematics, create_pinocchio_context
from nav_planning.planning import create_planner
from nav_planning.types import PlannerConfig

ROBOTS = [locobot, ridgeback]


@pytest.mark.parametrize("robot", ROBOTS)
def test_home_joints_has_five_dof(robot):
    assert robot.HOME_JOINTS.shape == (5,)


@pytest.mark.parametrize("robot", ROBOTS)
def test_joint_groups_cover_all_dof(robot):
    idx = set()
    for group_slice in robot.JOINT_GROUPS.values():
        idx.update(range(*group_slice.indices(5)))
    assert idx == set(range(5))


@pytest.mark.parametrize("robot", ROBOTS)
def test_planning_subgroups_exposed(robot):
    name = robot.ROBOT_NAME
    assert set(robot.PLANNING_SUBGROUPS) == {f"{name}_base", f"{name}_whole_body"}
    joints = robot.PLANNING_SUBGROUPS[f"{name}_whole_body"]["joints"]
    assert joints[:3] == ["base_x_joint", "base_y_joint", "base_theta_joint"]


@pytest.mark.parametrize("robot", ROBOTS)
def test_compiled_model_matches_constants(robot):
    planner = OmplVampPlanner(robot.ROBOT_NAME)
    assert planner.dimension() == 5
    lower, upper = planner.default_joint_limits()
    assert lower.shape == upper.shape == (5,)
    assert np.all(lower < upper)
    assert planner.validate(robot.HOME_JOINTS)


@pytest.mark.parametrize("robot", ROBOTS)
def test_low_level_api(robot, room):
    """The calls PHANTOM makes on autolife_planning's OmplVampPlanner."""
    planner = OmplVampPlanner(robot.ROBOT_NAME)
    lower, upper = planner.default_joint_limits()
    lower[:2], upper[:2] = (-1.0, -3.0), (7.0, 3.0)
    planner.set_joint_limits(lower, upper)
    np.testing.assert_array_equal(planner.joint_limits()[0], lower)
    planner.set_subgroup([0, 1, 2], robot.HOME_JOINTS.tolist())
    assert planner.dimension() == 3
    planner.add_pointcloud(room, 0.0)
    start = np.array(scene.START)
    goal = np.array(scene.GOALS["room 2"])
    result = planner.plan(
        start,
        goal,
        planner_name="rrtc",
        time_limit=2.0,
        simplify=True,
        interpolate=False,
    )
    assert result.solved
    assert isinstance(result.path, np.ndarray) and result.path.shape[1] == 3
    np.testing.assert_array_equal(result.path[0], start)
    np.testing.assert_array_equal(result.path[-1], goal)


def test_unknown_robot_raises():
    with pytest.raises(ValueError):
        OmplVampPlanner("fetch")


def test_partial_base_subgroup_raises():
    planner = OmplVampPlanner("locobot")
    with pytest.raises(ValueError):
        planner.set_subgroup([0, 1, 3], [0.0] * 5)


@pytest.mark.parametrize(
    "robot,heights",
    [(locobot, (0.4, 0.6)), (ridgeback, (1.1, 1.3))],
)
def test_camera_forward_kinematics(robot, heights):
    """The camera sits on top of the robot and turns with the pan joint."""
    config = getattr(robot, f"{robot.ROBOT_NAME}_robot_config")
    ctx = create_pinocchio_context(config.urdf_path, config.camera.link_name)
    q = robot.HOME_JOINTS.copy()
    pose = compute_forward_kinematics(ctx, q)
    assert heights[0] < pose.position[2] < heights[1]
    q[3] = np.pi / 2
    turned = compute_forward_kinematics(ctx, q)
    # Optical frame: z is the viewing direction.
    np.testing.assert_allclose(turned.rotation[:, 2], [0.0, 1.0, 0.0], atol=1e-6)


@pytest.mark.parametrize("subgroup", ["locobot_whole_body", "ridgeback_whole_body"])
def test_geometric_plan_is_collision_free(subgroup, room):
    """Densified along the state space's curves, every state is valid."""
    planner = create_planner(
        subgroup,
        config=PlannerConfig(time_limit=2.0, resolution=256.0),
        pointcloud=room,
    )
    planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)
    start = np.r_[scene.START, 0.0, 0.0]
    goal = np.r_[scene.GOALS["behind pillar"], 0.5, -0.2]
    result = planner.plan(start, goal)
    assert result.success
    assert planner.validate_batch(result.path).all()
