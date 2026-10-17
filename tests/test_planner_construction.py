import pytest

from nav_planning.planning import available_robots, create_planner
from nav_planning.types import PlannerConfig

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
    assert planner.has_base


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
