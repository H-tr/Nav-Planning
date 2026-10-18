/**
 * Main planner class — OMPL frontend with VAMP collision backend.
 *
 * ``OmplVampPlanner`` is the interface the Python extension exposes;
 * ``make_planner(robot)`` builds the ``Planner<Spec>`` of the named robot
 * ("locobot" or "ridgeback").  A planner starts on the full body (all
 * five joints); ``set_subgroup(active_indices, frozen_config)`` plans
 * over a subset with the other joints pinned.  The base joints
 * (indices 0, 1, 2) are either all active, leading the subgroup, or all
 * frozen.
 *
 * State spaces (see validity.hpp):
 *
 *   * with the base — the robot's SE(2) space (Reeds-Shepp for the
 *     LoCoBot's differential drive, plain SE(2) for the holonomic
 *     Ridgeback), composed with a RealVectorStateSpace for the camera
 *     joints;
 *   * without it — a RealVectorStateSpace.
 *
 * Every OMPL geometric planner runs on that space.
 *
 * Joint limits are kept for the full body and bound every active state
 * space, so they survive ``set_subgroup``.  ``base_theta_joint`` is
 * periodic in the SE(2) spaces; its limits are not used there.
 */

#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
// OMPL — informed trees
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/lazyinformedtrees/BLITstar.h>
// OMPL — FMT
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
// OMPL — KPIECE
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
// OMPL — PRM
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
// OMPL — RRT family
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
// OMPL — exploration-based
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vamp/collision/filter.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vector>

#include "plan_kinodynamic.hpp"
#include "validity.hpp"

namespace nav_planning {

namespace og = ompl::geometric;

/// (N, dof) waypoints and (N, 3) points, row-major so numpy arrays map
/// onto them without per-element conversion.
using Waypoints =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Points = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
/// (lower, upper) limits of every joint in full-body order.
using JointLimits = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

struct PlanResult {
  bool solved;
  Waypoints path;
  int64_t planning_time_ns;
  double path_cost;
};

class OmplVampPlanner {
 public:
  virtual ~OmplVampPlanner() = default;

  /// Set the scene pointcloud, replacing any previous one.
  virtual void add_pointcloud(const Eigen::Ref<const Points>& points,
                              float point_radius) = 0;
  virtual auto remove_pointcloud() -> bool = 0;
  virtual auto has_pointcloud() const -> bool = 0;
  virtual void add_sphere(const std::array<float, 3>& center, float radius) = 0;
  virtual void clear_environment() = 0;

  virtual auto plan(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                    const std::string& planner_name, double time_limit,
                    bool simplify, bool interpolate, double resolution)
      -> PlanResult = 0;
  virtual auto plan_kinodynamic(const std::vector<double>& start,
                                const std::vector<double>& start_velocity,
                                const std::vector<double>& goal,
                                double time_limit,
                                const KinodynamicSettings& settings)
      -> KinodynamicResult = 0;
  virtual auto simplify_path(const Waypoints& path, double time_limit)
      -> Waypoints = 0;
  virtual auto interpolate_path(const Waypoints& path, int count,
                                double resolution) -> Waypoints = 0;

  virtual auto validate(const Eigen::VectorXd& config) -> bool = 0;
  virtual auto validate_batch(const Eigen::Ref<const Waypoints>& configs)
      -> Eigen::Matrix<bool, Eigen::Dynamic, 1> = 0;
  virtual auto filter_pointcloud(const Eigen::Ref<const Points>& points,
                                 float min_dist, float max_range,
                                 const std::array<float, 3>& origin,
                                 const std::array<float, 3>& workspace_min,
                                 const std::array<float, 3>& workspace_max,
                                 bool cull) -> Points = 0;
  virtual auto filter_self_from_pointcloud(
      const Eigen::Ref<const Points>& points, float point_radius,
      const Eigen::VectorXd& config) -> Points = 0;

  virtual void set_subgroup(std::vector<int> active_indices,
                            std::vector<double> frozen_config) = 0;
  virtual auto dimension() const -> int = 0;
  virtual auto lower_bounds() const -> Eigen::VectorXd = 0;
  virtual auto upper_bounds() const -> Eigen::VectorXd = 0;
  virtual auto default_joint_limits() const -> JointLimits = 0;
  virtual auto joint_limits() const -> JointLimits = 0;
  virtual void set_joint_limits(const Eigen::VectorXd& lower,
                                const Eigen::VectorXd& upper) = 0;
};

template <typename Spec>
class Planner final : public OmplVampPlanner {
 public:
  using Robot = typename Spec::Robot;

  Planner() : joint_limits_(default_joint_limits()) {
    std::vector<int> all(Robot::dimension);
    for (std::size_t i = 0; i < Robot::dimension; ++i)
      all[i] = static_cast<int>(i);
    set_subgroup(all, std::vector<double>(Robot::dimension, 0.0));
  }

  void add_pointcloud(const Eigen::Ref<const Points>& points,
                      float point_radius) override {
    float_env_.pointclouds.clear();
    float_env_.pointclouds.emplace_back(to_vamp_points(points),
                                        Robot::min_radius, Robot::max_radius,
                                        point_radius);
    sync_env();
  }

  auto remove_pointcloud() -> bool override {
    if (float_env_.pointclouds.empty()) return false;
    float_env_.pointclouds.clear();
    sync_env();
    return true;
  }

  auto has_pointcloud() const -> bool override {
    return !float_env_.pointclouds.empty();
  }

  void add_sphere(const std::array<float, 3>& center, float radius) override {
    float_env_.spheres.emplace_back(vamp::collision::Sphere<float>{
        center[0], center[1], center[2], radius});
    float_env_.sort();
    sync_env();
  }

  void clear_environment() override {
    float_env_ = FloatEnv{};
    env_ = VampEnv{};
  }

  auto plan(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
            const std::string& planner_name, double time_limit, bool simplify,
            bool interpolate, double resolution) -> PlanResult override {
    check_dim(start.size(), "plan: start");
    check_dim(goal.size(), "plan: goal");

    const ob::SpaceInformationPtr si = make_space_information();
    const ob::PlannerPtr planner = create_geometric_planner(si, planner_name);
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<> ompl_start(si->getStateSpace());
    ob::ScopedState<> ompl_goal(si->getStateSpace());
    write_state(ompl_start.get(), sub_.layout, start.data(), dim());
    write_state(ompl_goal.get(), sub_.layout, goal.data(), dim());
    pdef->setStartAndGoalStates(ompl_start, ompl_goal);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Return on the first exact solution, the optimizing planners too.
    auto ptc = ob::plannerOrTerminationCondition(
        ob::timedPlannerTerminationCondition(time_limit),
        ob::exactSolnPlannerTerminationCondition(pdef));
    const auto t0 = std::chrono::steady_clock::now();
    // OMPL also reports approximate solutions, which stop short of the goal.
    const bool solved =
        planner->solve(ptc) == ob::PlannerStatus::EXACT_SOLUTION;
    PlanResult result;
    result.planning_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t0)
            .count();
    result.solved = solved;
    if (!solved) {
      result.path = Waypoints(0, dim());
      result.path_cost = std::numeric_limits<double>::infinity();
      return result;
    }
    auto& path = *pdef->getSolutionPath()->as<og::PathGeometric>();
    if (simplify) og::PathSimplifier(si).simplifyMax(path);
    if (interpolate) densify(path, 0, resolution);
    result.path_cost = path.length();
    result.path = to_waypoints(path);
    // The ends are start and goal; report them as given (SO(2) keeps the
    // yaw in [-pi, pi)).
    result.path.row(0) = start.transpose();
    result.path.row(result.path.rows() - 1) = goal.transpose();
    return result;
  }

  // Kinodynamic planning (FLASK, plan_kinodynamic.hpp): a
  // time-parameterised trajectory in the differentially flat output
  // space.  ``start_velocity`` is empty (start at rest) or an active-DOF
  // velocity in the layout the trajectory reports (base: world-frame
  // x_dot, y_dot, theta_dot); the goal is reached at rest.
  auto plan_kinodynamic(const std::vector<double>& start,
                        const std::vector<double>& start_velocity,
                        const std::vector<double>& goal, double time_limit,
                        const KinodynamicSettings& settings)
      -> KinodynamicResult override {
    check_dim(static_cast<Eigen::Index>(start.size()),
              "plan_kinodynamic: start");
    check_dim(static_cast<Eigen::Index>(goal.size()), "plan_kinodynamic: goal");
    if (!start_velocity.empty())
      check_dim(static_cast<Eigen::Index>(start_velocity.size()),
                "plan_kinodynamic: start_velocity");
    const int base_dim = sub_.layout == Layout::kJoints ? 0 : 3;
    if (static_cast<int>(settings.max_velocity.size()) != dim() - base_dim ||
        static_cast<int>(settings.max_acceleration.size()) != dim() - base_dim)
      throw std::invalid_argument(
          "plan_kinodynamic: settings need a velocity and an acceleration "
          "limit for each of the " +
          std::to_string(dim() - base_dim) + " non-base joints.");
    const auto lo = lower_bounds(), hi = upper_bounds();
    FlaskPlanner<Spec> flask(env_, sub_.active, sub_.frozen, base_dim,
                             std::vector<double>(lo.data(), lo.data() + dim()),
                             std::vector<double>(hi.data(), hi.data() + dim()),
                             settings);
    return flask.plan(start, start_velocity, goal, time_limit);
  }

  // Standalone path simplification against the current environment —
  // the shortcutting ``plan(..., simplify=True)`` runs.
  auto simplify_path(const Waypoints& path, double time_limit)
      -> Waypoints override {
    if (path.rows() < 2) return path;
    const auto si = make_space_information();
    og::PathGeometric geo = to_path(path, si);
    og::PathSimplifier(si).simplify(geo, time_limit);
    return to_waypoints(geo);
  }

  // Standalone densification along the state space's own interpolation
  // (Reeds-Shepp curves for the LoCoBot base); no collision checks.
  auto interpolate_path(const Waypoints& path, int count, double resolution)
      -> Waypoints override {
    if (path.rows() < 2) return path;
    const auto si = make_space_information();
    og::PathGeometric geo = to_path(path, si);
    densify(geo, count, resolution);
    return to_waypoints(geo);
  }

  auto validate(const Eigen::VectorXd& config) -> bool override {
    check_dim(config.size(), "validate: config");
    const auto q = full_config(config.data());
    return vamp::planning::validate_motion<Robot, kRake, 1>(q, q, env_);
  }

  // Batched collision check: ``kRake`` configurations per SIMD block, one
  // ``fkcc`` call each — the primitive the motion validator uses for the
  // samples of an edge.  A failed block is re-checked lane by lane so
  // every configuration gets its own answer.
  auto validate_batch(const Eigen::Ref<const Waypoints>& configs)
      -> Eigen::Matrix<bool, Eigen::Dynamic, 1> override {
    const auto n = static_cast<std::size_t>(configs.rows());
    Eigen::Matrix<bool, Eigen::Dynamic, 1> result =
        Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(configs.rows());
    if (n == 0) return result;
    check_dim(configs.cols(), "validate_batch: configs");
    for (std::size_t i = 0; i < n; i += kRake) {
      const std::size_t chunk = std::min(kRake, n - i);
      const bool block_valid = samples_valid<Robot>(
          env_, static_cast<int>(chunk), [&](int lane, float* q) {
            sub_.expand(configs.row(i + lane).data(), q);
          });
      for (std::size_t lane = 0; lane < chunk; ++lane)
        result[i + lane] =
            block_valid || validate(configs.row(i + lane).transpose());
    }
    return result;
  }

  /// Spatial down-sampling via Morton-curve sorting.
  auto filter_pointcloud(const Eigen::Ref<const Points>& points, float min_dist,
                         float max_range, const std::array<float, 3>& origin,
                         const std::array<float, 3>& workspace_min,
                         const std::array<float, 3>& workspace_max, bool cull)
      -> Points override {
    const auto filtered = vamp::collision::filter_pointcloud(
        to_vamp_points(points), min_dist, max_range,
        {origin[0], origin[1], origin[2]},
        {workspace_min[0], workspace_min[1], workspace_min[2]},
        {workspace_max[0], workspace_max[1], workspace_max[2]}, cull);
    Points out(static_cast<Eigen::Index>(filtered.size()), 3);
    for (std::size_t i = 0; i < filtered.size(); ++i)
      out.row(static_cast<Eigen::Index>(i)) << filtered[i][0], filtered[i][1],
          filtered[i][2];
    return out;
  }

  /// Remove points that collide with the robot body or the environment.
  auto filter_self_from_pointcloud(const Eigen::Ref<const Points>& points,
                                   float point_radius,
                                   const Eigen::VectorXd& config)
      -> Points override {
    check_dim(config.size(), "filter_self_from_pointcloud: config");
    const auto full = full_config(config.data()).to_array();
    typename Robot::template ConfigurationBlock<1> block;
    for (std::size_t i = 0; i < Robot::dimension; ++i) block[i] = full[i];
    typename Robot::template Spheres<1> spheres;
    Robot::template sphere_fk<1>(block, spheres);

    std::vector<Eigen::Index> keep;
    for (Eigen::Index p = 0; p < points.rows(); ++p) {
      const float x = points(p, 0), y = points(p, 1), z = points(p, 2);
      bool valid =
          !vamp::sphere_environment_in_collision(env_, x, y, z, point_radius);
      for (std::size_t i = 0; valid && i < Robot::n_spheres; ++i)
        valid = vamp::collision::sphere_sphere_sql2(
                    spheres.x[{i, 0}], spheres.y[{i, 0}], spheres.z[{i, 0}],
                    spheres.r[{i, 0}], x, y, z, point_radius) >= 0;
      if (valid) keep.push_back(p);
    }
    Points out(static_cast<Eigen::Index>(keep.size()), 3);
    for (std::size_t i = 0; i < keep.size(); ++i)
      out.row(static_cast<Eigen::Index>(i)) = points.row(keep[i]);
    return out;
  }

  /// Plan over ``active_indices`` with the other joints pinned to
  /// ``frozen_config`` (full body).  The environment is kept.
  void set_subgroup(std::vector<int> active_indices,
                    std::vector<double> frozen_config) override {
    auto sorted = active_indices;
    std::sort(sorted.begin(), sorted.end());
    if (frozen_config.size() != Robot::dimension ||
        std::any_of(sorted.begin(), sorted.end(),
                    [](int i) {
                      return i < 0 || i >= static_cast<int>(Robot::dimension);
                    }) ||
        std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
      throw std::invalid_argument(
          "set_subgroup: frozen_config needs " +
          std::to_string(Robot::dimension) +
          " entries and active_indices must index them, each once.");
    const auto base =
        std::count_if(active_indices.begin(), active_indices.end(),
                      [](int i) { return i < 3; });
    if (base != 0 && (base != 3 || active_indices[0] != 0 ||
                      active_indices[1] != 1 || active_indices[2] != 2))
      throw std::invalid_argument(
          "set_subgroup: the base joints 0, 1, 2 must be active together, "
          "leading the subgroup, or not at all.");
    sub_.active = std::move(active_indices);
    sub_.frozen.assign(frozen_config.begin(), frozen_config.end());
    sub_.layout = base == 0 ? Layout::kJoints
                  : static_cast<int>(sub_.active.size()) == 3
                      ? Layout::kBase
                      : Layout::kBaseJoints;
    build_space();
  }

  auto dimension() const -> int override { return dim(); }

  auto lower_bounds() const -> Eigen::VectorXd override {
    Eigen::VectorXd lo(dim());
    for (int i = 0; i < dim(); ++i) lo[i] = joint_limits_.first[sub_.active[i]];
    if (sub_.layout != Layout::kJoints) lo[2] = -M_PI;
    return lo;
  }

  auto upper_bounds() const -> Eigen::VectorXd override {
    Eigen::VectorXd hi(dim());
    for (int i = 0; i < dim(); ++i)
      hi[i] = joint_limits_.second[sub_.active[i]];
    if (sub_.layout != Layout::kJoints) hi[2] = M_PI;
    return hi;
  }

  /// The limits compiled into the robot model, in full-body order.
  auto default_joint_limits() const -> JointLimits override {
    std::array<float, Robot::dimension> zeros{}, ones{};
    ones.fill(1.0f);
    typename Robot::Configuration lo(zeros.data()), hi(ones.data());
    Robot::scale_configuration(lo);
    Robot::scale_configuration(hi);
    const auto lo_arr = lo.to_array(), hi_arr = hi.to_array();
    JointLimits limits{Eigen::VectorXd(Robot::dimension),
                       Eigen::VectorXd(Robot::dimension)};
    for (std::size_t i = 0; i < Robot::dimension; ++i) {
      limits.first[i] = std::min(lo_arr[i], hi_arr[i]);
      limits.second[i] = std::max(lo_arr[i], hi_arr[i]);
    }
    return limits;
  }

  auto joint_limits() const -> JointLimits override { return joint_limits_; }

  /// Replace the limits of every joint (full-body order).  Each joint
  /// needs finite bounds with lower < upper.
  void set_joint_limits(const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper) override {
    if (lower.size() != Robot::dimension || upper.size() != Robot::dimension)
      throw std::invalid_argument("set_joint_limits: lower and upper need " +
                                  std::to_string(Robot::dimension) +
                                  " entries.");
    for (Eigen::Index i = 0; i < lower.size(); ++i)
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
          !(lower[i] < upper[i]))
        throw std::invalid_argument("set_joint_limits: joint " +
                                    std::to_string(i) +
                                    " needs finite bounds with lower < upper.");
    joint_limits_ = {lower, upper};
    build_space();
  }

 private:
  Subgroup<Spec> sub_;
  JointLimits joint_limits_;
  ob::StateSpacePtr space_;
  FloatEnv float_env_;
  VampEnv env_;

  auto dim() const -> int { return static_cast<int>(sub_.active.size()); }

  void sync_env() { env_ = VampEnv(float_env_); }

  void check_dim(Eigen::Index n, const char* what) const {
    if (n != dim())
      throw std::invalid_argument(std::string(what) + " has " +
                                  std::to_string(n) + " entries, expected " +
                                  std::to_string(dim()) + ".");
  }

  static auto to_vamp_points(const Eigen::Ref<const Points>& points)
      -> std::vector<vamp::collision::Point> {
    std::vector<vamp::collision::Point> out(
        static_cast<std::size_t>(points.rows()));
    for (Eigen::Index i = 0; i < points.rows(); ++i)
      out[static_cast<std::size_t>(i)] = {points(i, 0), points(i, 1),
                                          points(i, 2)};
    return out;
  }

  auto full_config(const double* values) const ->
      typename Robot::Configuration {
    alignas(Robot::Configuration::S::Alignment)
        std::array<float, Robot::Configuration::num_scalars_rounded>
            q{};
    sub_.expand(values, q.data());
    return typename Robot::Configuration(q.data());
  }

  /// The robot's SE(2) space with the base x / y limits.
  auto make_base_space() const -> std::shared_ptr<ob::SE2StateSpace> {
    std::shared_ptr<ob::SE2StateSpace> se2;
    if constexpr (Spec::holonomic) {
      se2 = std::make_shared<ob::SE2StateSpace>();
    } else {
      se2 = std::make_shared<ob::ReedsSheppStateSpace>(Spec::turning_radius);
    }
    ob::RealVectorBounds bounds(2);
    for (int i = 0; i < 2; ++i) {
      bounds.setLow(i, joint_limits_.first[i]);
      bounds.setHigh(i, joint_limits_.second[i]);
    }
    se2->setBounds(bounds);
    return se2;
  }

  void build_space() {
    if (sub_.layout == Layout::kBase) {
      space_ = make_base_space();
      return;
    }
    const int first = sub_.layout == Layout::kJoints ? 0 : 3;
    auto joints = std::make_shared<ob::RealVectorStateSpace>(dim() - first);
    ob::RealVectorBounds bounds(dim() - first);
    for (int i = first; i < dim(); ++i) {
      bounds.setLow(i - first, joint_limits_.first[sub_.active[i]]);
      bounds.setHigh(i - first, joint_limits_.second[sub_.active[i]]);
    }
    joints->setBounds(bounds);
    if (sub_.layout == Layout::kJoints) {
      space_ = joints;
      return;
    }
    auto compound = std::make_shared<ob::CompoundStateSpace>();
    compound->addSubspace(make_base_space(), 1.0);
    compound->addSubspace(joints, 1.0);
    compound->lock();
    space_ = compound;
  }

  auto make_space_information() const -> ob::SpaceInformationPtr {
    auto si = std::make_shared<ob::SpaceInformation>(space_);
    si->setStateValidityChecker(
        std::make_shared<SubgroupValidityChecker<Spec>>(si, env_, sub_));
    si->setMotionValidator(
        std::make_shared<SubgroupMotionValidator<Spec>>(si, env_, sub_));
    si->setup();
    return si;
  }

  void densify(og::PathGeometric& path, int count, double resolution) const {
    if (count > 0 && resolution > 0.0)
      throw std::invalid_argument(
          "interpolate: pass at most one of count (>0) or resolution (>0).");
    if (count > 0) {
      path.interpolate(static_cast<unsigned int>(count));
    } else if (resolution > 0.0) {
      // Each edge of state-space length d gets ceil(d * resolution)
      // segments, along the space's own interpolation.
      auto& states = path.getStates();
      std::vector<ob::State*> dense{states.front()};
      for (std::size_t i = 1; i < states.size(); ++i) {
        const double d = space_->distance(states[i - 1], states[i]);
        const int n = std::max(1, static_cast<int>(std::ceil(d * resolution)));
        for (int k = 1; k < n; ++k) {
          auto* s = space_->allocState();
          space_->interpolate(states[i - 1], states[i],
                              static_cast<double>(k) / n, s);
          dense.push_back(s);
        }
        dense.push_back(states[i]);
      }
      states = std::move(dense);
    } else {
      path.interpolate();
    }
  }

  auto to_path(const Waypoints& waypoints,
               const ob::SpaceInformationPtr& si) const -> og::PathGeometric {
    check_dim(waypoints.cols(), "path waypoints");
    og::PathGeometric path(si);
    auto* s = space_->allocState();
    for (Eigen::Index i = 0; i < waypoints.rows(); ++i) {
      write_state(s, sub_.layout, waypoints.row(i).data(), dim());
      path.append(s);
    }
    space_->freeState(s);
    return path;
  }

  auto to_waypoints(const og::PathGeometric& path) const -> Waypoints {
    Waypoints out(static_cast<Eigen::Index>(path.getStateCount()), dim());
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
      read_state(path.getState(i), sub_.layout,
                 out.row(static_cast<Eigen::Index>(i)).data(), dim());
    return out;
  }

  static auto create_geometric_planner(const ob::SpaceInformationPtr& si,
                                       const std::string& name)
      -> ob::PlannerPtr {
    if (name == "rrtc" || name == "rrtconnect")
      return std::make_shared<og::RRTConnect>(si);
    if (name == "rrt") return std::make_shared<og::RRT>(si);
    if (name == "rrtstar") return std::make_shared<og::RRTstar>(si);
    if (name == "informed_rrtstar")
      return std::make_shared<og::InformedRRTstar>(si);
    if (name == "rrtsharp") return std::make_shared<og::RRTsharp>(si);
    if (name == "rrtxstatic") return std::make_shared<og::RRTXstatic>(si);
    if (name == "strrtstar") return std::make_shared<og::STRRTstar>(si);
    if (name == "lbtrrt") return std::make_shared<og::LBTRRT>(si);
    if (name == "trrt") return std::make_shared<og::TRRT>(si);
    if (name == "bitrrt") return std::make_shared<og::BiTRRT>(si);
    if (name == "bitstar") return std::make_shared<og::BITstar>(si);
    if (name == "abitstar") return std::make_shared<og::ABITstar>(si);
    if (name == "aitstar") return std::make_shared<og::AITstar>(si);
    if (name == "eitstar") return std::make_shared<og::EITstar>(si);
    if (name == "blitstar") return std::make_shared<og::BLITstar>(si);
    if (name == "fmt") return std::make_shared<og::FMT>(si);
    if (name == "bfmt") return std::make_shared<og::BFMT>(si);
    if (name == "kpiece") return std::make_shared<og::KPIECE1>(si);
    if (name == "bkpiece") return std::make_shared<og::BKPIECE1>(si);
    if (name == "lbkpiece") return std::make_shared<og::LBKPIECE1>(si);
    if (name == "prm") return std::make_shared<og::PRM>(si);
    if (name == "prmstar") return std::make_shared<og::PRMstar>(si);
    if (name == "lazyprm") return std::make_shared<og::LazyPRM>(si);
    if (name == "lazyprmstar") return std::make_shared<og::LazyPRMstar>(si);
    if (name == "spars") return std::make_shared<og::SPARS>(si);
    if (name == "spars2") return std::make_shared<og::SPARStwo>(si);
    if (name == "est") return std::make_shared<og::EST>(si);
    if (name == "biest") return std::make_shared<og::BiEST>(si);
    if (name == "sbl") return std::make_shared<og::SBL>(si);
    if (name == "stride") return std::make_shared<og::STRIDE>(si);
    if (name == "pdst") return std::make_shared<og::PDST>(si);
    throw std::invalid_argument("Unknown planner: " + name);
  }
};

/// The planner of ``robot``: "locobot" or "ridgeback".
inline auto make_planner(const std::string& robot)
    -> std::unique_ptr<OmplVampPlanner> {
  if (robot == "locobot") return std::make_unique<Planner<Locobot>>();
  if (robot == "ridgeback") return std::make_unique<Planner<Ridgeback>>();
  throw std::invalid_argument("Unknown robot '" + robot +
                              "': use 'locobot' or 'ridgeback'.");
}

}  // namespace nav_planning
