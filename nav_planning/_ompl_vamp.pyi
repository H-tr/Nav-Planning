"""
OMPL + VAMP C++ planning extension for the LoCoBot and the Ridgeback (5 DOF: planar base + camera pan and tilt).
"""

from collections.abc import Sequence
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

class PlanResult:
    def __init__(self, /, *args, **kwargs):
        """Initialize self.  See help(type(self)) for accurate signature."""

    @property
    def solved(self) -> bool: ...
    @property
    def path(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="C")]: ...
    @property
    def planning_time_ns(self) -> int: ...
    @property
    def path_cost(self) -> float: ...

class KinodynamicSettings:
    def __init__(self) -> None: ...
    @property
    def max_velocity(self) -> list[float]: ...
    @max_velocity.setter
    def max_velocity(self, arg: Sequence[float], /) -> None: ...
    @property
    def max_acceleration(self) -> list[float]: ...
    @max_acceleration.setter
    def max_acceleration(self, arg: Sequence[float], /) -> None: ...
    @property
    def base_max_speed(self) -> float: ...
    @base_max_speed.setter
    def base_max_speed(self, arg: float, /) -> None: ...
    @property
    def base_max_acceleration(self) -> float: ...
    @base_max_acceleration.setter
    def base_max_acceleration(self, arg: float, /) -> None: ...
    @property
    def base_max_yaw_rate(self) -> float: ...
    @base_max_yaw_rate.setter
    def base_max_yaw_rate(self, arg: float, /) -> None: ...
    @property
    def base_max_yaw_acceleration(self) -> float: ...
    @base_max_yaw_acceleration.setter
    def base_max_yaw_acceleration(self, arg: float, /) -> None: ...
    @property
    def allow_reverse(self) -> bool: ...
    @allow_reverse.setter
    def allow_reverse(self, arg: bool, /) -> None: ...
    @property
    def rho(self) -> float: ...
    @rho.setter
    def rho(self, arg: float, /) -> None: ...
    @property
    def limit_aware_duration(self) -> bool: ...
    @limit_aware_duration.setter
    def limit_aware_duration(self, arg: bool, /) -> None: ...
    @property
    def max_extension_time(self) -> float: ...
    @max_extension_time.setter
    def max_extension_time(self, arg: float, /) -> None: ...
    @property
    def velocity_sample_scale(self) -> float: ...
    @velocity_sample_scale.setter
    def velocity_sample_scale(self, arg: float, /) -> None: ...
    @property
    def velocity_metric_weight(self) -> float: ...
    @velocity_metric_weight.setter
    def velocity_metric_weight(self, arg: float, /) -> None: ...
    @property
    def rest_sample_probability(self) -> float: ...
    @rest_sample_probability.setter
    def rest_sample_probability(self, arg: float, /) -> None: ...
    @property
    def spin_probability(self) -> float: ...
    @spin_probability.setter
    def spin_probability(self, arg: float, /) -> None: ...
    @property
    def heading_tolerance(self) -> float: ...
    @heading_tolerance.setter
    def heading_tolerance(self, arg: float, /) -> None: ...
    @property
    def simplify(self) -> bool: ...
    @simplify.setter
    def simplify(self, arg: bool, /) -> None: ...
    @property
    def simplify_iterations(self) -> int: ...
    @simplify_iterations.setter
    def simplify_iterations(self, arg: int, /) -> None: ...
    @property
    def simplify_time_limit(self) -> float: ...
    @simplify_time_limit.setter
    def simplify_time_limit(self, arg: float, /) -> None: ...
    @property
    def max_iterations(self) -> int: ...
    @max_iterations.setter
    def max_iterations(self, arg: int, /) -> None: ...
    @property
    def seed(self) -> int: ...
    @seed.setter
    def seed(self, arg: int, /) -> None: ...

class FlatTrajectory:
    def __init__(self, /, *args, **kwargs):
        """Initialize self.  See help(type(self)) for accurate signature."""

    @property
    def duration(self) -> float: ...
    @property
    def active_dim(self) -> int: ...
    @property
    def base_dim(self) -> int: ...
    @property
    def num_segments(self) -> int: ...
    def position(
        self, t: float
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    def velocity(
        self, t: float
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    def acceleration(
        self, t: float
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    def sample(self, times: Sequence[float]) -> tuple[
        Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="F")],
        Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="F")],
        Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="F")],
    ]: ...
    def sample_uniform(self, dt: float) -> tuple[
        Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
        Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="F")],
        Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="F")],
        Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="F")],
    ]: ...
    def base_twist(self, t: float) -> list[float]: ...
    def knot_times(self) -> list[float]: ...
    def segment_kinds(self) -> list[int]: ...
    def segment_gears(self) -> list[int]: ...

class KinodynamicResult:
    def __init__(self, /, *args, **kwargs):
        """Initialize self.  See help(type(self)) for accurate signature."""

    @property
    def solved(self) -> bool: ...
    @property
    def trajectory(self) -> FlatTrajectory: ...
    @property
    def planning_time_ns(self) -> int: ...
    @property
    def simplify_time_ns(self) -> int: ...
    @property
    def cost(self) -> float: ...
    @property
    def iterations(self) -> int: ...
    @property
    def start_tree_size(self) -> int: ...
    @property
    def goal_tree_size(self) -> int: ...
    @property
    def edges_checked(self) -> int: ...

def flat_optimal_time(
    y0: Sequence[float],
    v0: Sequence[float],
    y1: Sequence[float],
    v1: Sequence[float],
    weights: Sequence[float],
    rho: float,
) -> tuple[float, float]:
    """
    Minimum-cost duration T* and cost J(T*) of the LQMT (cubic) motion between two flat states, J(T) = sum_i w_i int y_i''^2 dt + rho T.
    """

class OmplVampPlanner:
    @overload
    def __init__(self, robot: str) -> None:
        """
        Full-body planner (all five joints) of ``robot``: "locobot" or "ridgeback".
        """

    @overload
    def __init__(
        self, robot: str, active_indices: Sequence[int], frozen_config: Sequence[float]
    ) -> None:
        """
        Subgroup planner of ``robot`` over ``active_indices``; the other joints are pinned to ``frozen_config`` (full body).
        """

    def add_pointcloud(
        self,
        points: Annotated[
            NDArray[numpy.float32], dict(shape=(None, 3), writable=False)
        ],
        point_radius: float,
    ) -> None: ...
    def remove_pointcloud(self) -> bool: ...
    def has_pointcloud(self) -> bool: ...
    def add_sphere(self, center: Sequence[float], radius: float) -> None: ...
    def clear_environment(self) -> None: ...
    def plan(
        self,
        start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
        goal: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
        planner_name: str = "rrtc",
        time_limit: float = 10.0,
        simplify: bool = True,
        interpolate: bool = True,
        resolution: float = 64.0,
    ) -> PlanResult: ...
    def plan_kinodynamic(
        self,
        start: Sequence[float],
        start_velocity: Sequence[float],
        goal: Sequence[float],
        time_limit: float,
        settings: KinodynamicSettings,
    ) -> KinodynamicResult:
        """
        FLASK kinodynamic planning in the flat output space.  Returns a KinodynamicResult whose trajectory is time-parameterised.
        """

    def simplify_path(
        self,
        path: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="C")],
        time_limit: float = 1.0,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="C")]: ...
    def interpolate_path(
        self,
        path: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="C")],
        count: int = 0,
        resolution: float = 64.0,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order="C")]: ...
    def validate(
        self, config: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]
    ) -> bool: ...
    def validate_batch(
        self,
        configs: Annotated[
            NDArray[numpy.float64], dict(shape=(None, None), writable=False)
        ],
    ) -> Annotated[NDArray[numpy.bool_], dict(shape=(None,), order="C")]: ...
    def filter_pointcloud(
        self,
        points: Annotated[
            NDArray[numpy.float32], dict(shape=(None, 3), writable=False)
        ],
        min_dist: float,
        max_range: float,
        origin: Sequence[float],
        workspace_min: Sequence[float],
        workspace_max: Sequence[float],
        cull: bool = True,
    ) -> Annotated[NDArray[numpy.float32], dict(shape=(None, 3), order="C")]: ...
    def filter_self_from_pointcloud(
        self,
        points: Annotated[
            NDArray[numpy.float32], dict(shape=(None, 3), writable=False)
        ],
        point_radius: float,
        config: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
    ) -> Annotated[NDArray[numpy.float32], dict(shape=(None, 3), order="C")]: ...
    def set_subgroup(
        self, active_indices: Sequence[int], frozen_config: Sequence[float]
    ) -> None: ...
    def dimension(self) -> int: ...
    def lower_bounds(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    def upper_bounds(
        self,
    ) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")]: ...
    def default_joint_limits(
        self,
    ) -> tuple[
        Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
        Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
    ]: ...
    def joint_limits(
        self,
    ) -> tuple[
        Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
        Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
    ]: ...
    def set_joint_limits(
        self,
        lower: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
        upper: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
    ) -> None: ...
