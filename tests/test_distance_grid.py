"""The ESDF input: ``set_distance_grid`` on every robot."""

import numpy as np
import pytest

from nav_planning._ompl_vamp import OmplVampPlanner

ROBOTS = ["locobot", "ridgeback", "tortobot"]


def lattice(lower, upper, spacing):
    """Lattice points from ``lower`` to ``upper`` (m) and the grid's origin."""
    axes = [np.arange(lo, hi + spacing / 2, spacing) for lo, hi in zip(lower, upper)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1), [a[0] for a in axes]


def base_planner(robot):
    """The base subgroup of ``robot``, its camera joints at zero."""
    dof = OmplVampPlanner(robot).dimension()
    return OmplVampPlanner(robot, [0, 1, 2], [0.0] * dof)


@pytest.mark.parametrize("robot", ["ridgeback", "tortobot"])
def test_distance_grid_agrees_with_sphere_obstacles(robot):
    """Away from the surface, a grid of the spheres' distance accepts and rejects
    what the spheres do.

    The spheres lie far apart, so trilinear interpolation of their distance only
    overestimates it, by at most 3 h^2 / (8 d) in a cell at distance d or more
    from a centre: under 1 mm here (h = 2 cm, d >= 0.15 m). Configurations
    clear of the spheres by ``tol`` are free in the grid; those that overlap
    them by more than ``tol`` collide.
    """
    # TortoBot's sphere lies at z = 0; the obstacles of the Ridgeback reach its
    # deck and mast.
    height = {"ridgeback": [1.0, 0.4, 1.5], "tortobot": [0.0, 0.1, -0.1]}[robot]
    centers = np.array([[0.8, 0.0], [-0.6, 0.7], [0.0, -0.9]])
    centers = np.column_stack([centers, height])
    radius, spacing, tol = 0.15, 0.02, 2e-3
    points, origin = lattice([-3.0, -3.0, -0.5], [3.0, 3.0, 2.5], spacing)
    distances = np.linalg.norm(points[..., None, :] - centers, axis=-1).min(-1) - radius
    grid = base_planner(robot)
    grid.set_distance_grid(distances.astype(np.float32), origin, [spacing] * 3)
    inflated, deflated = base_planner(robot), base_planner(robot)
    for center in centers:
        inflated.add_sphere(center.tolist(), radius + tol)
        deflated.add_sphere(center.tolist(), radius - tol)

    rng = np.random.default_rng(0)
    q = rng.uniform([-1.0, -1.0, -np.pi], [1.0, 1.0, np.pi], (4000, 3))
    valid, free, colliding = (
        grid.validate_batch(q),
        inflated.validate_batch(q),
        ~deflated.validate_batch(q),
    )
    assert valid[free].all()
    assert not valid[colliding].any()
    assert free.any() and colliding.any()


def test_distance_grid_box_is_the_workspace():
    """A base whose sphere leaves the grid's box collides."""
    spacing = 0.05
    points, origin = lattice([-1.0, -1.0, -0.5], [1.0, 1.0, 0.5], spacing)
    planner = base_planner("tortobot")
    planner.set_distance_grid(
        np.full(points.shape[:3], 10.0, dtype=np.float32), origin, [spacing] * 3
    )
    assert planner.validate(np.array([0.0, 0.0, 0.0]))
    # The sphere (0.32 m) reaches past x = 1 m.
    assert planner.validate(np.array([0.67, 0.0, 0.0]))
    assert not planner.validate(np.array([0.69, 0.0, 0.0]))
    assert not planner.validate(np.array([1.5, 0.0, 0.0]))


@pytest.mark.parametrize("robot", ROBOTS)
def test_distance_grid_inputs_are_checked(robot):
    planner = base_planner(robot)
    field = np.zeros((4, 5, 6), dtype=np.float32)
    origin, spacing = [0.0, 0.0, 0.0], [0.1, 0.1, 0.1]
    assert not planner.has_distance_grid()
    with pytest.raises(ValueError):
        planner.set_distance_grid(field, origin, [0.1, 0.0, 0.1])
    with pytest.raises(ValueError):
        planner.set_distance_grid(field[:1].copy(), origin, spacing)
    with pytest.raises(TypeError):
        planner.set_distance_grid(field.astype(np.float64), origin, spacing)
    with pytest.raises(TypeError):
        planner.set_distance_grid(field[..., 0].copy(), origin, spacing)
    assert not planner.has_distance_grid()
    planner.set_distance_grid(field, origin, spacing)
    assert planner.has_distance_grid()
    assert planner.remove_distance_grid()
    assert not planner.remove_distance_grid()
    planner.set_distance_grid(field, origin, spacing)
    planner.clear_environment()
    assert not planner.has_distance_grid()


def test_distance_grid_reads_the_array_in_place():
    """The planner reads the caller's array without a copy and keeps it alive."""
    spacing = 0.05
    points, origin = lattice([-1.0, -1.0, -0.5], [1.0, 1.0, 0.5], spacing)
    field = np.full(points.shape[:3], 10.0, dtype=np.float32)
    planner = base_planner("tortobot")
    planner.set_distance_grid(field, origin, [spacing] * 3)
    assert planner.validate(np.array([0.0, 0.0, 0.0]))
    field[...] = -1.0
    assert not planner.validate(np.array([0.0, 0.0, 0.0]))
    del field
    assert not planner.validate(np.array([0.0, 0.0, 0.0]))


def test_distance_grid_replaces_the_last_one():
    spacing = 0.05
    points, origin = lattice([-1.0, -1.0, -0.5], [1.0, 1.0, 0.5], spacing)
    planner = base_planner("tortobot")
    planner.set_distance_grid(
        np.full(points.shape[:3], -1.0, dtype=np.float32), origin, [spacing] * 3
    )
    assert not planner.validate(np.array([0.0, 0.0, 0.0]))
    planner.set_distance_grid(
        np.full(points.shape[:3], 10.0, dtype=np.float32), origin, [spacing] * 3
    )
    assert planner.validate(np.array([0.0, 0.0, 0.0]))
