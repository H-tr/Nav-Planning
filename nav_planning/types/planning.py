from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from nav_planning.trajectory import KinodynamicTrajectory

# OMPL geometric planners, by family.
GEOMETRIC_PLANNERS = (
    # RRT family
    "rrtc",
    "rrt",
    "rrtstar",
    "informed_rrtstar",
    "rrtsharp",
    "rrtxstatic",
    "strrtstar",
    "lbtrrt",
    "trrt",
    "bitrrt",
    # Informed trees (asymptotically optimal)
    "bitstar",
    "abitstar",
    "aitstar",
    "eitstar",
    "blitstar",
    # FMT
    "fmt",
    "bfmt",
    # KPIECE
    "kpiece",
    "bkpiece",
    "lbkpiece",
    # PRM family
    "prm",
    "prmstar",
    "lazyprm",
    "lazyprmstar",
    "spars",
    "spars2",
    # Exploration-based
    "est",
    "biest",
    "sbl",
    "stride",
    "pdst",
)
# OMPL multilevel planners (SE(2) -> SE(2) x R^N); subgroups with the base.
MULTILEVEL_PLANNERS = ("qrrt", "qrrtstar", "qmp", "qmpstar")


class PlanningStatus(Enum):
    """Status of a motion planning attempt."""

    SUCCESS = "success"
    FAILED = "failed"
    INVALID_START = "invalid_start"
    INVALID_GOAL = "invalid_goal"


@dataclass
class PlannerConfig:
    """Configuration parameters for the motion planner."""

    planner_name: str = "rrtc"
    time_limit: float = 10.0
    # Radius of every obstacle point.  The robot's spheres already enclose
    # its meshes, so points need no extent of their own.
    point_radius: float = 0.0
    simplify: bool = True
    interpolate: bool = True
    # Interpolation density knobs (at most one may be nonzero):
    #   interpolate_count > 0 : total waypoint count (distance-weighted
    #       distribution across edges).
    #   resolution > 0.0      : waypoints per unit state-space distance
    #       — each edge of length d gets ceil(d * resolution) segments.
    #   both 0                : OMPL default longest-valid-segment
    #       fraction.
    interpolate_count: int = 0
    resolution: float = 64.0

    def __post_init__(self):
        if self.planner_name not in GEOMETRIC_PLANNERS + MULTILEVEL_PLANNERS:
            raise ValueError(
                f"Unknown planner '{self.planner_name}'. Supported: "
                f"{', '.join(GEOMETRIC_PLANNERS + MULTILEVEL_PLANNERS)}"
            )
        if self.time_limit <= 0:
            raise ValueError("time_limit must be > 0")
        if self.point_radius < 0:
            raise ValueError("point_radius must be >= 0")
        if self.interpolate_count < 0:
            raise ValueError("interpolate_count must be >= 0")
        if self.resolution < 0:
            raise ValueError("resolution must be >= 0")
        if self.interpolate_count > 0 and self.resolution > 0:
            raise ValueError(
                "Specify at most one of interpolate_count (>0) or "
                "resolution (>0), not both."
            )


@dataclass
class PlanningResult:
    """Result of a motion planning attempt."""

    status: PlanningStatus
    path: np.ndarray | None
    planning_time_ns: int
    iterations: int
    path_cost: float

    @property
    def success(self) -> bool:
        return self.status == PlanningStatus.SUCCESS


@dataclass
class KinodynamicConfig:
    """Parameters for :meth:`~nav_planning.planning.MotionPlanner.plan_kinodynamic`.

    The planner is FLASK (flatness-based kinodynamic RRT-Connect).  Joint
    and base limits default to the robot values in
    :mod:`nav_planning.locobot` / :mod:`nav_planning.ridgeback`;
    ``velocity_scale`` / ``acceleration_scale`` shrink or grow all of them
    at once.
    """

    time_limit: float = 1.0
    # LQMT time weight: effort weights are 1 / a_max^2 per flat output,
    # so rest-to-rest edges peak at sqrt(rho) x the acceleration limit.
    rho: float = 1.0
    velocity_scale: float = 1.0
    acceleration_scale: float = 1.0
    # Diff-drive (LoCoBot) reverse gear.
    allow_reverse: bool = True
    # Stretch T* until the exact velocity / acceleration bounds hold.
    limit_aware_duration: bool = True
    # Tree growth.
    max_extension_time: float = 3.0
    velocity_sample_scale: float = 0.5
    velocity_metric_weight: float = 0.3
    rest_sample_probability: float = 0.3
    spin_probability: float = 0.3
    heading_tolerance: float = 0.02
    # Post-processing after vamp::planning::simplify: greedy shortcuts and
    # subdivide-then-cut-corners smoothing, repeated until neither helps.
    simplify: bool = True
    simplify_iterations: int = 5
    simplify_time_limit: float = 0.05
    max_iterations: int = 200000
    # 0 = nondeterministic.
    seed: int = 0


@dataclass
class KinodynamicResult:
    """Result of a kinodynamic planning attempt.

    On success ``trajectory`` is a
    :class:`~nav_planning.trajectory.KinodynamicTrajectory` — already
    time-parameterised, so no separate time-parameterisation step is
    needed.
    """

    status: PlanningStatus
    trajectory: KinodynamicTrajectory | None
    planning_time_ns: int
    iterations: int
    cost: float
    simplify_time_ns: int = 0
    start_tree_size: int = 0
    goal_tree_size: int = 0
    edges_checked: int = 0

    @property
    def success(self) -> bool:
        return self.status == PlanningStatus.SUCCESS
