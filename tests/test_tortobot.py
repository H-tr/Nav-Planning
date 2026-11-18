"""TortoBot: PHANTOM's planar robot, one sphere of 0.32 m on a planar map."""

import numpy as np
import pytest

from nav_planning import tortobot
from nav_planning._ompl_vamp import OmplVampPlanner
from nav_planning.planning import create_planner

RADIUS = 0.32
# A planar map as an ESDF: three equal layers 1 m apart, so the sphere at
# z = 0 (radius 0.32 m) lies inside the lattice's box.
LAYERS = np.array([-1.0, 0.0, 1.0])


def planar_grid(field, origin, spacing):
    """The arguments of ``set_distance_grid`` for a planar field ``(X, Y)``."""
    layered = np.repeat(field[..., None], len(LAYERS), -1).astype(np.float32)
    return layered, [origin[0], origin[1], LAYERS[0]], [spacing, spacing, 1.0]


def base_planner():
    return OmplVampPlanner("tortobot", [0, 1, 2], [0.0] * 4)


def test_model():
    planner = OmplVampPlanner("tortobot")
    assert planner.dimension() == 4
    lower, upper = planner.default_joint_limits()
    np.testing.assert_allclose(lower, [-10.0, -10.0, -np.pi, -np.pi], rtol=1e-6)
    np.testing.assert_allclose(upper, [10.0, 10.0, np.pi, np.pi], rtol=1e-6)
    assert tortobot.tortobot_robot_config.joint_names == [
        "base_x_joint",
        "base_y_joint",
        "base_theta_joint",
        "camera_pan",
    ]
    assert create_planner("tortobot").num_dof == 4
    assert create_planner("tortobot_base").num_dof == 3


def test_one_sphere_of_the_disc_radius():
    """A wall at x = 1 m: the base is free at 0.33 m from it, not at 0.31 m,
    whatever its heading and the camera's angle."""
    spacing = 0.05
    xs, ys = np.arange(-2.0, 2.0 + 1e-9, spacing), np.arange(-2.0, 2.0 + 1e-9, spacing)
    field = np.broadcast_to((1.0 - xs)[:, None], (len(xs), len(ys)))
    planner = OmplVampPlanner("tortobot")
    planner.set_distance_grid(*planar_grid(field, [xs[0], ys[0]], spacing))
    for theta in np.linspace(-np.pi, np.pi, 9):
        for pan in (-2.0, 0.0, 2.5):
            assert planner.validate(np.array([1.0 - 0.33, 0.3, theta, pan]))
            assert not planner.validate(np.array([1.0 - 0.31, 0.3, theta, pan]))


def bilinear(field, origin, spacing, xy):
    """The bilinear value of a planar lattice field at points ``xy``."""
    c = (xy - origin) / spacing
    i = np.clip(np.floor(c).astype(int), 0, np.array(field.shape) - 2)
    a = c - i
    f = field
    return (
        f[i[:, 0], i[:, 1]] * (1 - a[:, 0]) * (1 - a[:, 1])
        + f[i[:, 0] + 1, i[:, 1]] * a[:, 0] * (1 - a[:, 1])
        + f[i[:, 0], i[:, 1] + 1] * (1 - a[:, 0]) * a[:, 1]
        + f[i[:, 0] + 1, i[:, 1] + 1] * a[:, 0] * a[:, 1]
    )


def test_planar_map_is_exact():
    """On a planar ESDF the sphere is free exactly where the bilinear map value
    at the base is at least 0.32 m: PHANTOM's test of its disc."""
    rng = np.random.default_rng(1)
    spacing = 0.1
    nx, ny = 61, 41
    origin = np.array([0.0, 0.0])
    field = rng.uniform(-0.2, 1.0, (nx, ny)).astype(np.float32)
    planner = base_planner()
    planner.set_distance_grid(*planar_grid(field, origin, spacing))
    lo, hi = origin + RADIUS, origin + spacing * (np.array([nx, ny]) - 1) - RADIUS
    xy = rng.uniform(lo, hi, (20000, 2))
    q = np.column_stack([xy, rng.uniform(-np.pi, np.pi, len(xy))])
    value = bilinear(field.astype(np.float64), origin, spacing, xy)
    clear = np.abs(value - RADIUS) > 1e-4
    valid = planner.validate_batch(q)
    np.testing.assert_array_equal(valid[clear], (value >= RADIUS)[clear])
    assert 0.2 < valid.mean() < 0.8


def wall_with_gap():
    """A planar ESDF of a wall at x = 2 m with a 1.0 m gap centred at y = 1.5 m."""
    spacing = 0.05
    xs = np.arange(0.0, 4.0 + 1e-9, spacing)
    ys = np.arange(0.0, 3.0 + 1e-9, spacing)
    x, y = np.meshgrid(xs, ys, indexing="ij")
    # The wall is the segment x = 2, outside 1.0 <= y <= 2.0.
    below = np.hypot(x - 2.0, np.maximum(y - 1.0, 0.0))
    above = np.hypot(x - 2.0, np.minimum(y - 2.0, 0.0))
    return np.minimum(below, above) - 0.05, [xs[0], ys[0]], spacing


def test_rrt_connect_through_a_gap():
    field, origin, spacing = wall_with_gap()
    planner = base_planner()
    planner.set_joint_limits(
        np.array([RADIUS, RADIUS, -np.pi, -np.pi]),
        np.array([4.0 - RADIUS, 3.0 - RADIUS, np.pi, np.pi]),
    )
    planner.set_distance_grid(*planar_grid(field, origin, spacing))
    start, goal = np.array([0.6, 0.6, 0.0]), np.array([3.4, 2.4, 3.0])
    for _ in range(10):
        result = planner.plan(
            start,
            goal,
            planner_name="rrtc",
            time_limit=1.0,
            simplify=True,
            interpolate=False,
        )
        assert result.solved
        path = result.path
        np.testing.assert_array_equal(path[0], start)
        np.testing.assert_array_equal(path[-1], goal)
        # Every edge, sampled densely along the SE(2) interpolation
        # (straight x, y and the shortest arc of the heading), is free.
        for a, b in zip(path[:-1], path[1:]):
            t = np.linspace(0.0, 1.0, 200)[:, None]
            turn = np.remainder(b[2] - a[2] + np.pi, 2 * np.pi) - np.pi
            samples = np.column_stack(
                [a[:2] + t * (b[:2] - a[:2]), a[2] + t[:, 0] * turn]
            )
            assert planner.validate_batch(samples).all()


@pytest.mark.parametrize("subgroup", ["tortobot_base", "tortobot_whole_body"])
def test_kinodynamic_in_free_space(subgroup):
    from nav_planning.types import KinodynamicConfig

    planner = create_planner(subgroup)
    planner.set_joint_limits({"base_x_joint": (-1.0, 4.0), "base_y_joint": (-1.0, 4.0)})
    field = np.full((51, 51), 5.0)
    planner.set_distance_grid(*planar_grid(field, [-1.0, -1.0], 0.1))
    n = planner.num_dof
    start, goal = np.zeros(n), np.array([2.0, 1.0, 1.0, 0.5][:n])
    result = planner.plan_kinodynamic(
        start, goal, config=KinodynamicConfig(time_limit=2.0)
    )
    assert result.success
