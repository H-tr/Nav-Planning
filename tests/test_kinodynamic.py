"""Kinodynamic (FLASK) planning: closed-form math and end-to-end checks.

Every plan is verified independently of the planner's own validation:
the returned trajectory is densely resampled and checked for collisions
(``validate_batch``), joint / base limits, C^1 continuity, and — for the
LoCoBot's differential drive — the nonholonomic constraint (zero lateral
velocity).
"""

import numpy as np
import pytest
import scene

from nav_planning import locobot, ridgeback
from nav_planning._ompl_vamp import flat_optimal_time
from nav_planning.planning import create_planner
from nav_planning.types import KinodynamicConfig, PlannerConfig, PlanningStatus

DT = 0.002
TOL = 1e-6
ROBOT = {"locobot": locobot, "ridgeback": ridgeback}


def lqmt_cost(y0, v0, y1, v1, w, rho, T):
    """J(T) of the Hermite cubic, by numerical quadrature."""
    y0, v0, y1, v1, w = map(np.asarray, (y0, v0, y1, v1, w))
    d = y1 - y0
    c2 = (3 * d - T * (2 * v0 + v1)) / T**2
    c3 = (-2 * d + T * (v0 + v1)) / T**3
    t = np.linspace(0.0, T, 20001)[:, None]
    acc = 2 * c2 + 6 * c3 * t
    return float(np.trapezoid((w * acc**2).sum(axis=1), t[:, 0]) + rho * T)


def assert_same_config(a, b, atol=1e-6):
    """Compare configurations, treating the base heading modulo 2 pi."""
    a, b = np.asarray(a, dtype=float).copy(), np.asarray(b, dtype=float).copy()
    assert abs(np.angle(np.exp(1j * (a[2] - b[2])))) <= atol
    a[2] = b[2] = 0.0
    np.testing.assert_allclose(a, b, atol=atol)


def body_frame(theta, vx, vy):
    """World-frame planar vectors in the base frame."""
    c, s = np.cos(theta), np.sin(theta)
    return c * vx + s * vy, -s * vx + c * vy


def assert_executable(planner, result, start, goal, start_velocity=None):
    """Independent checks of a returned kinodynamic trajectory."""
    assert result.success, result.status
    robot = ROBOT[planner.robot_name.split("_")[0]]
    traj = result.trajectory
    assert traj.duration > 0.0 and traj.has_base
    t, q, qd, qdd = traj.sample_uniform(DT)

    # Boundary conditions: exact start state, goal reached at rest.  The
    # LoCoBot's yaw rate is acceleration-level for its flat outputs: it is
    # finite (not necessarily zero) when the base starts or stops, and a
    # replanned trajectory may start with a different yaw rate — like at
    # any knot.  All flat rates must match exactly.
    assert_same_config(q[0], start)
    assert_same_config(q[-1], goal)
    holonomic = robot is ridgeback
    flat = [i for i in range(q.shape[1]) if holonomic or i != 2]
    np.testing.assert_allclose(qd[-1][flat], 0.0, atol=1e-6)
    if start_velocity is None:
        np.testing.assert_allclose(qd[0][flat], 0.0, atol=1e-6)
    else:
        np.testing.assert_allclose(
            qd[0][flat], np.asarray(start_velocity)[flat], atol=1e-6
        )

    # Collision-free along the executed trajectory (not just at nodes).
    assert planner.validate_batch(q).all()

    # C^1 in the flat outputs: positions and velocities continuous across
    # segment knots.
    eps = 1e-7
    for k in traj.knot_times[1:-1]:
        assert_same_config(traj.position(k - eps), traj.position(k + eps), 1e-5)
        np.testing.assert_allclose(
            traj.velocity(k - eps)[flat], traj.velocity(k + eps)[flat], atol=1e-4
        )

    # Camera joint limits.
    names = planner.joint_names[3:]
    vel = np.array([robot.JOINT_VELOCITY_LIMITS[j] for j in names])
    acc = np.array([robot.JOINT_ACCELERATION_LIMITS[j] for j in names])
    lo = planner._planner.lower_bounds()[3:]
    hi = planner._planner.upper_bounds()[3:]
    assert np.all(q[:, 3:] >= lo - TOL) and np.all(q[:, 3:] <= hi + TOL)
    assert np.all(np.abs(qd[:, 3:]) <= vel * (1 + 1e-6) + TOL)
    assert np.all(np.abs(qdd[:, 3:]) <= acc * (1 + 1e-6) + TOL)

    # Base limits.  Sampled checks in the planner run on a 50 ms grid;
    # allow a small overshoot between grid points.
    theta = q[:, 2]
    twist = traj.sample_base_twist(t)
    vx_b, vy_b = body_frame(theta, qd[:, 0], qd[:, 1])
    np.testing.assert_allclose(twist[:, 0], vx_b, atol=1e-9)
    np.testing.assert_allclose(twist[:, 1], vy_b, atol=1e-9)
    assert np.abs(twist[:, :2]).max() <= robot.BASE_MAX_SPEED * (1 + 1e-3)
    assert np.abs(twist[:, 2]).max() <= robot.BASE_MAX_YAW_RATE * 1.05
    if holonomic:
        # The controller limits each body axis of the commanded velocity
        # and of its rate of change.
        dv = np.gradient(twist[:, :2], t, axis=0)
        assert np.abs(dv[2:-2]).max() <= robot.BASE_MAX_ACCELERATION * 1.05
        assert np.abs(qdd[:, 2]).max() <= robot.BASE_MAX_YAW_ACCELERATION * (1 + 1e-6)
    else:
        # Exactly nonholonomic: no lateral velocity.
        np.testing.assert_allclose(vy_b, 0.0, atol=1e-9)
        a_tan = np.gradient(twist[:, 0], t)
        assert np.abs(a_tan[2:-2]).max() <= robot.BASE_MAX_ACCELERATION * 1.05
    # Heading continuity (no teleporting base).
    dtheta = np.abs(np.angle(np.exp(1j * np.diff(theta))))
    assert dtheta.max() <= robot.BASE_MAX_YAW_RATE * DT * 2 + 1e-3


# ── Closed-form LQMT ───────────────────────────────────────────────────


def test_optimal_time_rest_to_rest_closed_form():
    # r = 2, rest-to-rest, unit weight: T* = (36 d^2 / rho)^(1/4).
    for d, rho in [(1.0, 1.0), (0.3, 4.0), (2.5, 0.5)]:
        T, J = flat_optimal_time([0.0], [0.0], [d], [0.0], [1.0], rho)
        assert T == pytest.approx((36 * d**2 / rho) ** 0.25, rel=1e-9)
        assert J == pytest.approx(lqmt_cost([0], [0], [d], [0], [1], rho, T), rel=1e-6)


@pytest.mark.parametrize("seed", range(10))
def test_optimal_time_is_global_minimum(seed):
    rng = np.random.default_rng(seed)
    n = 5
    y0, y1 = rng.uniform(-2, 2, n), rng.uniform(-2, 2, n)
    v0, v1 = rng.uniform(-1, 1, n), rng.uniform(-1, 1, n)
    w = rng.uniform(0.2, 3.0, n)
    rho = float(rng.uniform(0.1, 10.0))
    T, J = flat_optimal_time(y0, v0, y1, v1, w, rho)
    grid = np.geomspace(1e-2, 1e2, 4000)
    d = y1 - y0
    a = np.sum(w * 12 * d**2)
    b = np.sum(w * 12 * (v0 + v1) * d)
    c = np.sum(w * 4 * (v0**2 + v0 * v1 + v1**2))
    costs = a / grid**3 - b / grid**2 + c / grid + rho * grid
    assert J <= costs.min() + 1e-9
    # Stationarity at T*.
    dJ = -3 * a / T**4 + 2 * b / T**3 - c / T**2 + rho
    assert abs(dJ) < 1e-6 * max(1.0, rho)
    assert J == pytest.approx(lqmt_cost(y0, v0, y1, v1, w, rho, T), rel=1e-5)


# ── The demo scene ─────────────────────────────────────────────────────

SUBGROUPS = [
    "locobot_base",
    "locobot_whole_body",
    "ridgeback_base",
    "ridgeback_whole_body",
]
CAMERA_GOAL = {"locobot": (0.8, 0.4), "ridgeback": (1.5, -0.5)}


def make_planner(subgroup, room):
    planner = create_planner(
        subgroup, config=PlannerConfig(time_limit=1.0), pointcloud=room
    )
    planner.set_base_bounds(-1.0, 7.0, -3.0, 3.0)
    return planner


def endpoints(subgroup, goal_name):
    start, goal = np.array(scene.START), np.array(scene.GOALS[goal_name])
    if subgroup.endswith("whole_body"):
        start = np.r_[start, 0.0, 0.0]
        goal = np.r_[goal, CAMERA_GOAL[subgroup.split("_")[0]]]
    return start, goal


@pytest.mark.parametrize("goal_name", scene.GOALS)
@pytest.mark.parametrize("subgroup", SUBGROUPS)
def test_demo_scene_plan_is_executable(subgroup, goal_name, room):
    planner = make_planner(subgroup, room)
    start, goal = endpoints(subgroup, goal_name)
    cfg = KinodynamicConfig(time_limit=3.0, seed=2)
    result = planner.plan_kinodynamic(start, goal, config=cfg)
    assert_executable(planner, result, start, goal)


@pytest.mark.parametrize("subgroup", ["locobot_whole_body", "ridgeback_whole_body"])
def test_plan_is_deterministic_with_seed(subgroup, room):
    planner = make_planner(subgroup, room)
    start, goal = endpoints(subgroup, "behind pillar")
    cfg = KinodynamicConfig(time_limit=3.0, seed=3)
    a = planner.plan_kinodynamic(start, goal, config=cfg)
    b = planner.plan_kinodynamic(start, goal, config=cfg)
    assert a.success and b.success
    np.testing.assert_allclose(a.trajectory.knot_times, b.trajectory.knot_times)


@pytest.mark.parametrize("subgroup", ["locobot_whole_body", "ridgeback_whole_body"])
def test_replans_while_driving(subgroup, room):
    planner = make_planner(subgroup, room)
    start, goal = endpoints(subgroup, "room 2")
    cfg = KinodynamicConfig(time_limit=3.0, seed=6)
    first = planner.plan_kinodynamic(start, goal, config=cfg)
    assert first.success
    tr = first.trajectory
    # Pick a time where the base is actually driving.
    t = np.linspace(0.0, tr.duration, 200)
    speed = np.hypot(*tr.sample_base_twist(t)[:, :2].T)
    t_mid = float(t[np.argmax(speed)])
    q_mid, qd_mid = tr.position(t_mid), tr.velocity(t_mid)
    _, new_goal = endpoints(subgroup, "table")
    second = planner.plan_kinodynamic(
        q_mid, new_goal, start_velocity=qd_mid, config=cfg
    )
    assert_executable(planner, second, q_mid, new_goal, start_velocity=qd_mid)


def test_locobot_forward_only(room):
    planner = make_planner("locobot_base", room)
    start, goal = np.array(scene.START), np.array([-0.5, 0.0, 0.0])
    cfg = KinodynamicConfig(time_limit=3.0, seed=4, allow_reverse=False)
    result = planner.plan_kinodynamic(start, goal, config=cfg)
    assert_executable(planner, result, start, goal)
    assert all(g == 0 for g in result.trajectory.segment_gears)
    t = np.arange(0.0, result.trajectory.duration, DT)
    assert result.trajectory.sample_base_twist(t)[:, 0].min() >= -1e-9


def test_ridgeback_moves_sideways(room):
    """A holonomic base reaches a pose to its side without turning."""
    planner = make_planner("ridgeback_base", room)
    start, goal = np.array(scene.START), np.array([0.0, 1.0, 0.0])
    result = planner.plan_kinodynamic(start, goal, config=KinodynamicConfig(seed=5))
    assert_executable(planner, result, start, goal)
    _, q, _, _ = result.trajectory.sample_uniform(DT)
    np.testing.assert_allclose(q[:, 2], 0.0, atol=1e-9)


def test_invalid_start_is_reported(room):
    planner = make_planner("locobot_base", room)
    inside_pillar = np.array([1.5, -1.5, 0.0])
    assert not planner.validate(inside_pillar)
    result = planner.plan_kinodynamic(inside_pillar, np.array(scene.START))
    assert result.status == PlanningStatus.INVALID_START
    assert result.trajectory is None and not result.success
