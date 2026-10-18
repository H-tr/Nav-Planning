import numpy as np
import pytest

from nav_planning._ompl_vamp import OmplVampPlanner
from nav_planning.planning import available_robots, create_planner
from nav_planning.types import PlannerConfig, PlanningStatus
from nav_planning.types.planning import GEOMETRIC_PLANNERS

SUBGROUPS = {
    "locobot": 5,
    "locobot_base": 3,
    "locobot_whole_body": 5,
    "ridgeback": 5,
    "ridgeback_base": 3,
    "ridgeback_whole_body": 5,
}


@pytest.mark.parametrize("subgroup", SUBGROUPS)
def test_planner_constructs(subgroup):
    planner = create_planner(
        subgroup, config=PlannerConfig(planner_name="rrtc", time_limit=0.1)
    )
    assert planner.num_dof == SUBGROUPS[subgroup]
    lower, upper = planner.bounds
    assert lower.shape == upper.shape == (planner.num_dof,)


def test_available_robots():
    assert set(available_robots()) == set(SUBGROUPS)


def test_unknown_robot_raises():
    with pytest.raises(ValueError):
        create_planner("fetch")


def test_set_subgroup_keeps_environment(room):
    planner = create_planner("locobot", pointcloud=room)
    planner.set_subgroup("locobot_base")
    assert planner.num_dof == 3 and planner.has_pointcloud
    with pytest.raises(ValueError):
        planner.set_subgroup("ridgeback_base")


def test_set_joint_limits_by_name():
    planner = create_planner("locobot")
    compiled = planner.joint_limits
    assert list(compiled) == planner.joint_names
    planner.set_joint_limits({"base_x_joint": (-1.0, 7.0), "tilt": (-0.5, 0.5)})
    limits = planner.joint_limits
    assert limits["base_x_joint"] == (-1.0, 7.0) and limits["tilt"] == (-0.5, 0.5)
    # Joints left out keep the compiled range.
    assert limits["base_y_joint"] == compiled["base_y_joint"]
    lower, upper = planner.bounds
    assert (lower[0], upper[0]) == (-1.0, 7.0) and (lower[4], upper[4]) == (-0.5, 0.5)
    # The heading of an active base is periodic.
    assert (lower[2], upper[2]) == (-np.pi, np.pi)
    # Kept across subgroups; None restores the compiled ranges.
    planner.set_subgroup("locobot_base")
    assert planner.joint_limits["base_x_joint"] == (-1.0, 7.0)
    planner.set_joint_limits(None)
    assert planner.joint_limits == compiled


@pytest.mark.parametrize(
    "limits", [{"elbow": (0.0, 1.0)}, {"pan": (1.0, -1.0)}, {"pan": (0.0, np.inf)}]
)
def test_set_joint_limits_rejects_and_keeps_the_ranges(limits):
    planner = create_planner("ridgeback")
    planner.set_joint_limits({"base_y_joint": (-3.0, 3.0)})
    before = planner.joint_limits
    with pytest.raises(ValueError):
        planner.set_joint_limits(limits)
    assert planner.joint_limits == before


def test_config_joint_limits():
    config = PlannerConfig(joint_limits={"base_x_joint": (0, 2)})
    assert config.joint_limits == {"base_x_joint": (0.0, 2.0)}
    planner = create_planner("ridgeback_base", config=config)
    assert planner.joint_limits["base_x_joint"] == (0.0, 2.0)
    for bad in ({"pan": (1.0,)}, {"pan": (0.0, np.nan)}, {"pan": (1.0, 1.0)}):
        with pytest.raises(ValueError):
            PlannerConfig(joint_limits=bad)


def test_plan_checks_its_inputs(room):
    planner = create_planner("locobot_base", pointcloud=room)
    with pytest.raises(ValueError):
        planner.plan(np.zeros(5), np.zeros(3))
    inside_pillar = np.array([1.5, -1.5, 0.0])
    result = planner.plan(np.zeros(3), inside_pillar)
    assert result.status == PlanningStatus.INVALID_GOAL
    assert result.path is None and result.path_cost == float("inf")


def test_sample_valid_is_within_bounds():
    planner = create_planner("ridgeback_whole_body")
    planner.set_joint_limits({"base_x_joint": (0.0, 1.0)})
    q = planner.sample_valid()
    lower, upper = planner.bounds
    assert np.all(q >= lower) and np.all(q <= upper) and planner.validate(q)


def test_strrtstar_is_not_offered():
    """OMPL's STRRT* needs a space-time state space, which no planner
    space here is."""
    with pytest.raises(ValueError):
        PlannerConfig(planner_name="strrtstar")
    with pytest.raises(ValueError):
        OmplVampPlanner("locobot").plan(
            np.zeros(5), np.full(5, 0.1), planner_name="strrtstar"
        )


# Known OMPL behaviour (docs/planning/index.md, "Supported planners").
INFORMED = ("informed_rrtstar", "bitstar", "abitstar", "aitstar", "eitstar", "blitstar")


def known_failure(subgroup, planner_name):
    if planner_name in INFORMED and subgroup.endswith("whole_body"):
        return pytest.mark.xfail(
            raises=RuntimeError, reason="OMPL's informed samplers reject SE(2) x R^N"
        )
    if planner_name == "blitstar":
        return pytest.mark.xfail(raises=RuntimeError, reason="OMPL BLIT* bug")
    if (planner_name, subgroup) in {
        ("fmt", "locobot_base"),
        ("fmt", "locobot_whole_body"),
        ("bfmt", "locobot_whole_body"),
    }:
        return pytest.mark.xfail(reason="OMPL's FMT* finds no LoCoBot path")
    return ()


@pytest.mark.parametrize(
    "subgroup,planner_name",
    [
        pytest.param(subgroup, name, marks=known_failure(subgroup, name))
        for subgroup in SUBGROUPS
        if subgroup.endswith(("_base", "_whole_body"))
        for name in GEOMETRIC_PLANNERS
    ],
)
def test_every_planner_plans(subgroup, planner_name):
    planner = create_planner(
        subgroup, config=PlannerConfig(planner_name=planner_name, time_limit=0.5)
    )
    planner.set_joint_limits({"base_x_joint": (-2.0, 2.0), "base_y_joint": (-2.0, 2.0)})
    start = np.zeros(planner.num_dof)
    goal = start.copy()
    goal[:3] = [1.0, 0.5, 0.3]
    assert planner.plan(start, goal).success
