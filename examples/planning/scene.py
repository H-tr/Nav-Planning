"""The demo scene: two rooms joined by a doorway, as a point cloud.

Room 1 (x < 3) holds a table (top at 0.75 m: the LoCoBot fits under it,
the Ridgeback's mast does not) and a pillar; room 2 (x > 3) holds a shelf
board 0.45 m above the floor (the LoCoBot's camera tower hits it) and a
second pillar.  The rooms are joined by a 1.2 m doorway in the dividing
wall at x = 3.  The floor is not part of the cloud: the robots' spheres
reach slightly below the floor plane.
"""

import numpy as np

SPACING = 0.025  # m between neighbouring points


def _box(lo, hi):
    """Points on the surface of the axis-aligned box [lo, hi]."""
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    axes = [
        np.linspace(a, b, max(2, int(np.ceil((b - a) / SPACING)) + 1))
        for a, b in zip(lo, hi)
    ]
    faces = []
    for k in range(3):
        for side in (lo[k], hi[k]):
            grid = np.meshgrid(
                *[ax if i != k else [side] for i, ax in enumerate(axes)], indexing="ij"
            )
            faces.append(np.stack(grid, -1).reshape(-1, 3))
    return np.unique(np.concatenate(faces), axis=0)


def _cylinder(x, y, radius, height):
    """Points on the side of a vertical cylinder standing on the floor."""
    n = int(np.ceil(2 * np.pi * radius / SPACING))
    angle = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    z = np.linspace(0.0, height, int(np.ceil(height / SPACING)) + 1)
    a, h = np.meshgrid(angle, z, indexing="ij")
    return np.stack([x + radius * np.cos(a), y + radius * np.sin(a), h], -1).reshape(
        -1, 3
    )


def room_pointcloud() -> np.ndarray:
    """The demo scene as an ``(N, 3)`` float32 point cloud."""
    wall = 1.5
    parts = [
        # Outer walls of the 8 m x 6 m space.
        _box([-1.0, -3.0, 0.0], [7.0, -3.0, wall]),
        _box([-1.0, 3.0, 0.0], [7.0, 3.0, wall]),
        _box([-1.0, -3.0, 0.0], [-1.0, 3.0, wall]),
        _box([7.0, -3.0, 0.0], [7.0, 3.0, wall]),
        # Dividing wall at x = 3 with a 1.2 m doorway.
        _box([3.0, -3.0, 0.0], [3.0, -0.6, wall]),
        _box([3.0, 0.6, 0.0], [3.0, 3.0, wall]),
        # Table: top 0.75 m above the floor, four legs.
        _box([1.0, 1.2, 0.72], [2.2, 2.0, 0.75]),
        *[
            _box([x, y, 0.0], [x + 0.05, y + 0.05, 0.72])
            for x in (1.0, 2.15)
            for y in (1.2, 1.95)
        ],
        # Shelf board 0.45 m above the floor along the far wall of room 2.
        _box([4.5, -3.0, 0.44], [5.5, -1.6, 0.45]),
        # Pillars.
        _cylinder(1.5, -1.5, 0.2, wall),
        _cylinder(5.0, 1.0, 0.2, wall),
    ]
    return np.concatenate(parts).astype(np.float32)


# The base's range on this scene, for ``MotionPlanner.set_joint_limits``:
# the 8 m x 6 m space inside the outer walls.
BASE_LIMITS = {"base_x_joint": (-1.0, 7.0), "base_y_joint": (-3.0, 3.0)}

# Base poses (x, y, theta) used by the examples, tests and benchmark.
START = (0.0, 0.0, 0.0)
GOALS = {
    # Through the doorway into room 2, turned around.
    "room 2": (6.0, 0.0, np.pi),
    # Behind the pillar in room 2, facing the wall.
    "behind pillar": (5.5, 1.9, np.pi / 2),
    # Next to the table in room 1, facing it.
    "table": (1.6, 0.7, np.pi / 2),
}
