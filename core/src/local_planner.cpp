// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/planning/local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/geometry/polynomial.hpp"

namespace fiss_planner {

EgoState sampleTrajectory(const Trajectory& traj, double t) {
  if (traj.size() == 0) return {};
  if (t <= traj.t.front()) return traj.cartesianStateAt(0);
  if (t >= traj.t.back()) return traj.cartesianStateAt(traj.size() - 1);
  const auto it = std::upper_bound(traj.t.begin(), traj.t.end(), t);
  const size_t i = static_cast<size_t>(it - traj.t.begin());
  const double w = (t - traj.t[i - 1]) / (traj.t[i] - traj.t[i - 1]);
  const auto lerp = [w](double a, double b) { return a + w * (b - a); };
  EgoState s;
  s.x = lerp(traj.x[i - 1], traj.x[i]);
  s.y = lerp(traj.y[i - 1], traj.y[i]);
  s.yaw = traj.yaw[i - 1] + w * normalizeAngle(traj.yaw[i] - traj.yaw[i - 1]);
  s.v = lerp(traj.v[i - 1], traj.v[i]);
  s.a = lerp(traj.a[i - 1], traj.a[i]);
  s.kappa = lerp(traj.kappa[i - 1], traj.kappa[i]);
  return s;
}

namespace {

/// Points of `traj` with t in [t_from, t_to), re-timed to start at 0.
Trajectory slice(const Trajectory& traj, double t_from, double t_to) {
  Trajectory out;
  for (size_t i = 0; i < traj.size(); ++i) {
    if (traj.t[i] < t_from - 1e-9 || traj.t[i] >= t_to - 1e-9) continue;
    out.t.push_back(traj.t[i] - t_from);
    for (auto [dst, src] : {std::pair{&out.s, &traj.s},
                            {&out.s_d, &traj.s_d},
                            {&out.s_dd, &traj.s_dd},
                            {&out.s_ddd, &traj.s_ddd},
                            {&out.d, &traj.d},
                            {&out.d_d, &traj.d_d},
                            {&out.d_dd, &traj.d_dd},
                            {&out.d_ddd, &traj.d_ddd},
                            {&out.x, &traj.x},
                            {&out.y, &traj.y},
                            {&out.yaw, &traj.yaw},
                            {&out.v, &traj.v},
                            {&out.a, &traj.a},
                            {&out.kappa, &traj.kappa}}) {
      dst->push_back((*src)[i]);
    }
  }
  out.end = traj.end;
  out.lane = traj.lane;
  out.cost = traj.cost;
  out.is_cartesian = traj.is_cartesian;
  return out;
}

/// Append `tail` (re-timed by `offset`) to `head`.
void append(Trajectory& head, const Trajectory& tail, double offset) {
  for (size_t i = 0; i < tail.size(); ++i) {
    head.t.push_back(tail.t[i] + offset);
    for (auto [dst, src] : {std::pair{&head.s, &tail.s},
                            {&head.s_d, &tail.s_d},
                            {&head.s_dd, &tail.s_dd},
                            {&head.s_ddd, &tail.s_ddd},
                            {&head.d, &tail.d},
                            {&head.d_d, &tail.d_d},
                            {&head.d_dd, &tail.d_dd},
                            {&head.d_ddd, &tail.d_ddd},
                            {&head.x, &tail.x},
                            {&head.y, &tail.y},
                            {&head.yaw, &tail.yaw},
                            {&head.v, &tail.v},
                            {&head.a, &tail.a},
                            {&head.kappa, &tail.kappa}}) {
      dst->push_back((*src)[i]);
    }
  }
}

// Free-space mode.
constexpr double kFreeSpaceEntrySpeed = 2.0;  // [m/s] enter only below this speed
constexpr double kBandSlack = 0.3;            // [m] inside a lane band up to this far beyond its edges
constexpr double kHandoverOffset = 0.5;       // [m] back in the lane this close to its center ...
constexpr double kHandoverHeading = 0.25;     // [rad] ... and this well aligned
constexpr double kReplanDeviation = 1.0;      // [m] replan this far off the path
constexpr double kCuspReach = 0.4;            // [m] a change of direction is reached this close ...
constexpr double kCuspSpeed = 0.2;            // [m/s] ... at this speed
constexpr double kSlowObstacle = 0.5;         // [m/s] slower obstacles are mapped as occupied
constexpr double kHoldTime = 2.0;             // [s] length of a standstill plan
constexpr double kGoalSearch = 10.0;          // [m] beyond goal_ahead to find a free goal

/// Standing still at the vehicle's pose.
Trajectory standstill(const EgoState& ego, double s, double dt) {
  Trajectory traj;
  for (double t : timeSamples(kHoldTime, dt)) {
    traj.t.push_back(t);
    traj.x.push_back(ego.x);
    traj.y.push_back(ego.y);
    traj.yaw.push_back(ego.yaw);
    traj.s.push_back(s);
  }
  const size_t n = traj.t.size();
  for (auto* v : {&traj.s_d, &traj.s_dd, &traj.s_ddd, &traj.d, &traj.d_d, &traj.d_dd, &traj.d_ddd, &traj.v,
                  &traj.a, &traj.kappa}) {
    v->assign(n, 0.0);
  }
  traj.end = {0.0, 0.0, kHoldTime};
  traj.is_cartesian = true;
  return traj;
}

std::vector<Vec2> lanePolygon(const MapLane& lane) {
  std::vector<Vec2> poly = lane.left;
  poly.insert(poly.end(), lane.right.rbegin(), lane.right.rend());
  return poly;
}

}  // namespace

LocalPlanner::LocalPlanner(const LocalPlannerConfig& config, const VehicleParams& vehicle)
    : config_(config),
      vehicle_(vehicle),
      planner_(makePlanner(config.planner, vehicle)),
      behavior_(std::make_unique<BehaviorPlanner>(config.behavior, vehicle)) {
  config_.validate();
}

void LocalPlanner::setMap(std::shared_ptr<const RoadMap> map, std::vector<int> route,
                          std::optional<Vec2> goal) {
  map_ = map ? std::make_shared<RoadMap>(*map) : nullptr;
  route_ = std::move(route);
  goal_ = goal;
  path_ref_.reset();
  reset();
}

void LocalPlanner::setReferencePath(const std::vector<double>& x, const std::vector<double>& y) {
  auto ref = std::make_shared<ReferenceLine>(x, y);
  const auto& p = config_.path;
  path_corridor_ =
      Corridor::parallelLanes(0.0, ref->length(), p.lane_width, p.num_left_lanes, p.num_right_lanes);
  for (auto& lane : path_corridor_.lanes()) {
    if (lane.offset > 0) lane.change_allowed = p.allow_left;
    if (lane.offset < 0) lane.change_allowed = p.allow_right;
  }
  path_ref_ = std::move(ref);
  map_.reset();
  route_.clear();
  goal_.reset();
  reset();
}

void LocalPlanner::requestConfig(const LocalPlannerConfig& config) {
  config.validate();
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_config_ = config;
}

void LocalPlanner::requestPlannerType(PlannerType type) {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  if (!pending_config_) pending_config_ = config_;
  pending_config_->planner.type = type;
}

void LocalPlanner::setStopLineActive(int stop_line_id, bool active, bool yellow) {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_stop_lines_.emplace_back(stop_line_id, active, yellow);
}

void LocalPlanner::reset() {
  previous_.reset();
  previous_emergency_ = false;
  previous_ego_lane_ = -1;
  free_path_.clear();
  free_index_ = 0;
  planner_->reset();
  behavior_->reset();
}

void LocalPlanner::applyPendingConfig() {
  std::optional<LocalPlannerConfig> next;
  std::vector<std::tuple<int, bool, bool>> stop_lines;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    next.swap(pending_config_);
    stop_lines.swap(pending_stop_lines_);
  }
  if (map_) {
    for (const auto& [id, active, yellow] : stop_lines) map_->setStopLineActive(id, active, yellow);
  }
  if (!next) return;
  const bool type_changed = next->planner.type != config_.planner.type;
  const bool path_changed = next->path.lane_width != config_.path.lane_width ||
                            next->path.num_left_lanes != config_.path.num_left_lanes ||
                            next->path.num_right_lanes != config_.path.num_right_lanes ||
                            next->path.allow_left != config_.path.allow_left ||
                            next->path.allow_right != config_.path.allow_right;
  config_ = *next;
  behavior_ = std::make_unique<BehaviorPlanner>(config_.behavior, vehicle_);
  if (type_changed) {
    planner_ = makePlanner(config_.planner, vehicle_);
  } else {
    planner_->setConfig(config_.planner);
  }
  if (path_changed && path_ref_) {
    const auto& p = config_.path;
    path_corridor_ =
        Corridor::parallelLanes(0.0, path_ref_->length(), p.lane_width, p.num_left_lanes, p.num_right_lanes);
    for (auto& lane : path_corridor_.lanes()) {
      if (lane.offset > 0) lane.change_allowed = p.allow_left;
      if (lane.offset < 0) lane.change_allowed = p.allow_right;
    }
  }
}

double LocalPlanner::targetSpeed(const ReferenceLine& ref, double s, double v, double desired,
                                 double lane_limit) const {
  double target = desired;
  if (config_.use_speed_limit && std::isfinite(lane_limit) && lane_limit > 0.0) {
    target = std::min(target, lane_limit);
  }
  if (config_.use_curvature_speed_limit) {
    const double horizon = std::max({v, target, 5.0}) * config_.planner.sampling.max_t;
    for (double ds = 0.0; ds <= horizon; ds += 2.0) {
      const double k = std::abs(ref.at(s + ds).kappa);
      if (k > 1e-4) target = std::min(target, std::sqrt(vehicle_.max_lat_accel / k));
    }
  }
  return std::min(target, vehicle_.max_speed);
}

Trajectory LocalPlanner::emergencyStop(const FrenetState& start, const ReferenceLine& ref) const {
  // Brake at the limit while bringing the lateral motion to rest at the
  // current offset (quintic over 1.5 s), until standstill plus one second.
  const double decel = config_.emergency_decel > 0.0 ? config_.emergency_decel : vehicle_.max_decel;
  const double dt = config_.planner.sampling.dt;
  const double v0 = std::max(0.0, start.s_d);
  const double t_stop = v0 / decel;
  const double T_lat = 1.5;
  const double d_end = start.d + start.d_d * 0.5 * T_lat;
  const QuinticPolynomial lat(start.d, start.d_d, start.d_dd, d_end, 0.0, 0.0, T_lat);
  Trajectory traj;
  traj.end = {d_end, 0.0, t_stop + 1.0};
  for (double t : timeSamples(std::max(t_stop + 1.0, T_lat), dt)) {
    const double tb = std::min(t, t_stop);
    traj.t.push_back(t);
    traj.s.push_back(start.s + v0 * tb - 0.5 * decel * tb * tb);
    traj.s_d.push_back(std::max(0.0, v0 - decel * t));
    traj.s_dd.push_back(t < t_stop ? -decel : 0.0);
    traj.s_ddd.push_back(0.0);
    const double tl = std::min(t, T_lat);
    traj.d.push_back(lat.x(tl));
    traj.d_d.push_back(t < T_lat ? lat.dx(tl) : 0.0);
    traj.d_dd.push_back(t < T_lat ? lat.ddx(tl) : 0.0);
    traj.d_ddd.push_back(t < T_lat ? lat.dddx(tl) : 0.0);
  }
  convertToCartesian(ref, traj);
  return traj;
}

bool LocalPlanner::freeSpaceStep(const EgoState& ego, const std::vector<Obstacle>& obstacles,
                                 LocalPlanOutput& out, bool allow_handover) {
  const FreeSpaceConfig& fc = config_.free_space;
  const ReferenceLine& ref = *out.reference;
  const Corridor& corridor = out.corridor;
  const LaneBand* ego_band = corridor.egoLane();
  if (!ego_band) {
    free_path_.clear();
    return false;
  }
  const FrenetState f = ref.toFrenet(ego);
  if (free_path_.empty()) {
    // Enter at low speed outside every lane of the corridor.
    if (std::abs(ego.v) > kFreeSpaceEntrySpeed) return false;
    for (const auto& lane : corridor.lanes()) {
      if (f.d <= corridor.left(lane, f.s) + kBandSlack && f.d >= corridor.right(lane, f.s) - kBandSlack)
        return false;
    }
  } else if (allow_handover && std::abs(f.d - corridor.center(*ego_band, f.s)) < kHandoverOffset &&
             std::abs(normalizeAngle(ego.yaw - ref.at(f.s).yaw)) < kHandoverHeading) {
    free_path_.clear();  // back in the lane
    return false;
  }

  // Goal on the ego lane's center, goal_ahead ahead (further if occupied).
  const auto lanePose = [&](double s) {
    const RefPoint p = ref.at(s);
    const double c = corridor.center(*ego_band, s);
    return FreeSpacePose{p.x - c * std::sin(p.yaw), p.y + c * std::cos(p.yaw), p.yaw, false};
  };
  const FreeSpacePose nominal = lanePose(f.s + fc.goal_ahead);

  // Grid around the vehicle and the goal: the map's lanes and areas, the
  // vehicle's own footprint, minus standing obstacles. Moving ones are
  // checked along the trajectory instead.
  OccupancyGrid grid(0.5 * (ego.x + nominal.x), 0.5 * (ego.y + nominal.y), fc.grid_size, fc.resolution);
  const double reach = fc.grid_size;  // lanes with a centerline point this close may cover the grid
  for (const auto& [id, lane] : map_->lanes()) {
    const bool near = std::any_of(lane.centerline.begin(), lane.centerline.end(), [&](const Vec2& p) {
      return std::hypot(p.x - ego.x, p.y - ego.y) < reach;
    });
    if (near) grid.markFree(lanePolygon(lane));
  }
  for (const auto& area : map_->areas()) grid.markFree(area);
  {
    Box2d own{ego.x, ego.y, ego.yaw, vehicle_.length + 2.0 * fc.margin, vehicle_.width + 2.0 * fc.margin};
    const auto c = own.corners();
    grid.markFree({c.begin(), c.end()});
  }
  std::vector<Obstacle> moving;
  for (const Obstacle& ob : obstacles) {
    if (ob.prediction.empty()) continue;
    if (ob.is_static || std::abs(ob.prediction.front().v) < kSlowObstacle) {
      if (const auto box = ob.boxAt(0.0)) grid.markOccupied(*box);
    } else {
      moving.push_back(ob);
    }
  }
  grid.computeClearance();

  const double dt = config_.planner.sampling.dt;
  out.free_space = true;
  const auto hold = [&](const char* why) {
    out.trajectory = standstill(ego, f.s, dt);
    out.behavior.emplace_back(why);
    out.free_space_path = free_path_;
    return true;
  };

  // Progress along the path: the nearest segment up to the next change of
  // direction, which is passed once reached at near standstill.
  bool replan = free_path_.empty();
  if (!replan) {
    const size_t n = free_path_.size();
    const bool dir = free_path_[std::min(free_index_ + 1, n - 1)].reverse;
    size_t cusp = std::min(free_index_ + 1, n - 1);
    while (cusp + 1 < n && free_path_[cusp + 1].reverse == dir) ++cusp;
    size_t nearest = free_index_;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = free_index_; i < cusp; ++i) {
      const FreeSpacePose& a = free_path_[i];
      const FreeSpacePose& b = free_path_[i + 1];
      const double len2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
      const double u =
          len2 > 1e-9
              ? std::clamp(((ego.x - a.x) * (b.x - a.x) + (ego.y - a.y) * (b.y - a.y)) / len2, 0.0, 1.0)
              : 0.0;
      const double d = std::hypot(a.x + u * (b.x - a.x) - ego.x, a.y + u * (b.y - a.y) - ego.y);
      if (d < best) {
        best = d;
        nearest = i;
      }
    }
    const FreeSpacePose& c = free_path_[cusp];
    if (cusp + 1 < n && std::hypot(c.x - ego.x, c.y - ego.y) < kCuspReach && std::abs(ego.v) < kCuspSpeed) {
      nearest = cusp;  // reached the change of direction
    }
    free_index_ = nearest;
    const FreeSpacePose& last = free_path_.back();
    if (cusp + 1 >= n && std::hypot(last.x - ego.x, last.y - ego.y) < kCuspReach &&
        std::abs(ego.v) < kCuspSpeed) {
      // At the end: the lane planner takes over from within the lane.
      const double half = 0.5 * (corridor.left(*ego_band, f.s) - corridor.right(*ego_band, f.s));
      if (allow_handover && std::abs(f.d - corridor.center(*ego_band, f.s)) < half) {
        free_path_.clear();
        out.free_space = false;
        return false;
      }
      replan = true;
    }
    replan = replan || best > kReplanDeviation || free_index_ + 1 >= n;
    // Obstacles that came into the way (the path may run closer than the
    // margin where it starts in a tight spot).
    for (size_t i = free_index_; !replan && i < n; ++i) {
      replan = !freeSpaceCollisionFree(grid, free_path_[i], vehicle_, 0.0);
    }
  }
  if (replan) {
    free_path_.clear();
    std::optional<FreeSpacePose> goal;
    for (double ahead = 0.0; ahead <= kGoalSearch && !goal; ahead += 1.0) {
      const FreeSpacePose g = lanePose(f.s + fc.goal_ahead + ahead);
      if (freeSpaceCollisionFree(grid, g, vehicle_, fc.margin)) goal = g;
    }
    const FreeSpacePose from{ego.x, ego.y, ego.yaw, false};
    std::optional<std::vector<FreeSpacePose>> path;
    if (goal) {
      // With half the margin, then none, if the full one leaves no way out
      // (e.g. parked close to the curb in a narrow parking lane).
      FreeSpaceConfig search = fc;
      for (const double scale : {1.0, 0.5, 0.0}) {
        search.margin = scale * fc.margin;
        path = planHybridAStar(grid, from, *goal, vehicle_, search);
        if (path || fc.margin == 0.0) break;
      }
    }
    if (!path) return hold("free_space_blocked");
    free_path_ = std::move(*path);
    free_index_ = 0;
  }

  // The rest of the path, from the vehicle's projection onto it.
  std::vector<FreeSpacePose> rest(free_path_.begin() + static_cast<long>(free_index_), free_path_.end());
  if (rest.size() > 1) {
    const FreeSpacePose& a = rest[0];
    const FreeSpacePose& b = rest[1];
    const double len2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    const double u =
        len2 > 1e-9 ? std::clamp(((ego.x - a.x) * (b.x - a.x) + (ego.y - a.y) * (b.y - a.y)) / len2, 0.0, 1.0)
                    : 0.0;
    rest[0] = {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.yaw + u * normalizeAngle(b.yaw - a.yaw),
               a.reverse};
  }
  const bool reverse = rest.size() > 1 && rest[1].reverse;
  Trajectory traj = freeSpaceTrajectory(rest, reverse ? -std::abs(ego.v) : std::abs(ego.v), fc, dt);
  // Wait for moving traffic to pass.
  int checks = 0;
  const CollisionChecker checker(config_.planner.collision, vehicle_);
  if (!checker.isCollisionFree(traj, moving, checks) ||
      !checker.keepsTimeGap(traj, moving, config_.planner.safety.crossing_time_gap, checks)) {
    return hold("free_space_yield");
  }
  out.success = true;
  out.trajectory = std::move(traj);
  out.behavior.emplace_back("free_space");
  out.free_space_path = free_path_;
  return true;
}

LocalPlanOutput LocalPlanner::step(double t_now, const EgoState& ego, const std::vector<Obstacle>& obstacles,
                                   double desired_speed) {
  applyPendingConfig();
  LocalPlanOutput out;
  const double previous_step_t = last_step_t_;
  last_step_t_ = t_now;
  const auto& st = config_.stitching;
  const double latency = st.enabled ? std::max(0.0, st.latency) : 0.0;

  // --- Start state (stitching) ------------------------------------------------
  EgoState start = ego;
  std::optional<Trajectory> prefix;
  if (previous_emergency_) {
    // Recover from an emergency stop from the measured state: stitching onto the
    // stop profile would carry its full braking into every new plan.
    start.a = std::max(start.a, -config_.recovery_decel);
  } else if (st.enabled && previous_ && previous_->size() > 1) {
    const double t_rel = t_now - previous_t0_;
    if (t_rel + latency <= previous_->t.back()) {
      const EgoState expected = sampleTrajectory(*previous_, t_rel);
      const double pos_err = std::hypot(expected.x - ego.x, expected.y - ego.y);
      const double yaw_err = std::abs(normalizeAngle(expected.yaw - ego.yaw));
      if (pos_err <= st.max_position_error && yaw_err <= st.max_heading_error) {
        start = sampleTrajectory(*previous_, t_rel + latency);
        if (latency > 0.0) prefix = slice(*previous_, t_rel, t_rel + latency);
        out.stitched = true;
      }
    }
  }

  // --- Reference and corridor -------------------------------------------------
  double lane_limit = std::numeric_limits<double>::infinity();
  std::vector<RouteCorridor::StopLineAhead> stop_line_info;
  std::vector<std::pair<double, double>> junctions;
  if (map_) {
    auto rc = buildRouteCorridor(*map_, route_, start, config_.route, previous_ego_lane_, goal_);
    if (!rc) {
      previous_.reset();
      return out;  // off the map
    }
    out.reference = std::make_shared<ReferenceLine>(std::move(rc->reference));
    out.corridor = std::move(rc->corridor);
    out.ego_lane = rc->ego_lane;
    out.virtual_obstacles = std::move(rc->stop_lines);
    stop_line_info = std::move(rc->stop_line_info);
    junctions = std::move(rc->junctions);
    lane_limit = rc->speed_limit;
    previous_ego_lane_ = rc->ego_lane;
  } else if (path_ref_) {
    out.reference = path_ref_;
    out.corridor = path_corridor_;
  } else {
    throw std::logic_error("LocalPlanner::step: call setMap() or setReferencePath() first");
  }
  const ReferenceLine& ref = *out.reference;

  out.start = ref.toFrenet(start);
  out.start.s_d = std::max(0.0, out.start.s_d);
  // A vehicle about to stop does not keep decelerating once at rest: below
  // 1 m/s, plans start with at most the deceleration (3 v / max_t) with which
  // stopping within the horizon does not back up.
  if (out.start.s_d < 1.0) {
    out.start.s_dd = std::max(out.start.s_dd, -3.0 * out.start.s_d / config_.planner.sampling.max_t);
  }
  const double desired = desired_speed > 0.0 ? desired_speed : config_.desired_speed;
  out.target_speed = targetSpeed(ref, out.start.s, start.v, desired, lane_limit);

  // --- Behaviours ------------------------------------------------------------------
  if (config_.behavior.enabled && map_) {
    BehaviorInput bi;
    bi.reference = &ref;
    bi.start = out.start;
    bi.stop_lines = &stop_line_info;
    bi.junctions = &junctions;
    bi.obstacles = &obstacles;
    bi.corridor = &out.corridor;
    if (std::isfinite(previous_step_t)) bi.dt = std::clamp(t_now - previous_step_t, 0.0, 1.0);
    const BehaviorDirective directive = behavior_->plan(bi);
    out.behavior = directive.active;
    for (auto& lane : out.corridor.lanes()) {
      if (lane.oncoming) {
        lane.change_allowed = directive.pass_blocked && directive.oncoming_allowed;
        lane.extra_cost = config_.behavior.oncoming_cost;
      } else if (lane.offset == 0 && directive.pass_blocked) {
        // Leave the lane, or wait in it with room to pull out.
        lane.extra_cost += config_.behavior.blocked_cost;
        const double v = std::max(0.0, out.start.s_d);
        const double can_stop =
            out.start.s + 0.5 * vehicle_.length + v * v / (2.0 * config_.behavior.comfort_decel);
        lane.end_s = std::max(directive.blocked_s - config_.behavior.pass_standoff, can_stop + 1.0);
      }
    }
    for (const int id : directive.released_stop_lines) {
      auto& v = out.virtual_obstacles;
      v.erase(std::remove_if(v.begin(), v.end(), [id](const Obstacle& o) { return o.id == -1000 - id; }),
              v.end());
    }
    const LaneBand* ego_band = nullptr;
    for (const auto& lane : out.corridor.lanes()) {
      if (lane.offset == 0) ego_band = &lane;
    }
    for (size_t k = 0; k < directive.stops.size(); ++k) {
      const RefPoint p = ref.at(directive.stops[k]);
      Obstacle ob;
      ob.id = -2000 - static_cast<int>(k);
      ob.length = 1.0;
      ob.width = ego_band
                     ? std::max(1.0, out.corridor.left(*ego_band, p.s) - out.corridor.right(*ego_band, p.s))
                     : 3.5;
      ob.is_static = true;
      ob.prediction.push_back({0.0, p.x, p.y, p.yaw, 0.0});
      out.virtual_obstacles.push_back(std::move(ob));
    }
  }

  // --- Free space (outside the lanes) ------------------------------------------
  // Right after handing over to the lane planner, the free-space plan stays
  // the fallback for this cycle (e.g. still alongside the parked cars).
  std::optional<LocalPlanOutput> handover;
  if (config_.free_space.enabled && map_) {
    std::vector<FreeSpacePose> path = free_path_;
    const size_t index = free_index_;
    if (freeSpaceStep(ego, obstacles, out)) {
      previous_ = out.trajectory;
      previous_t0_ = t_now;
      previous_emergency_ = false;
      return out;
    }
    if (!path.empty()) {
      std::swap(free_path_, path);
      free_index_ = index;
      LocalPlanOutput fs = out;
      if (freeSpaceStep(ego, obstacles, fs, false)) handover = std::move(fs);
      free_path_.clear();
    }
  }

  // --- Obstacles relative to the start time -----------------------------------
  std::vector<Obstacle> all = obstacles;
  all.insert(all.end(), out.virtual_obstacles.begin(), out.virtual_obstacles.end());
  if (latency > 0.0) {
    for (auto& ob : all) {
      if (ob.is_static) continue;
      for (auto& p : ob.prediction) p.t -= latency;
    }
  }

  PlanningInput in;
  in.start = out.start;
  in.reference = &ref;
  in.corridor = &out.corridor;
  in.obstacles = &all;
  in.target_speed = out.target_speed;
  PlanResult res = planner_->plan(in);
  out.stats = res.stats;
  out.candidates = std::move(res.candidates);

  // Keep executing what is left of the last plan if it is still collision
  // free; failing that, retry with the safety envelope as a cost instead of a
  // check (a close but collision-free plan beats hard braking in traffic, and
  // the one that breaches the envelope least wins), then with a shorter
  // collision horizon; otherwise brake. Planning starts afresh next cycle.
  std::optional<Trajectory> rest;
  if (!res.success && previous_) {
    const double t_rel = t_now - previous_t0_;
    if (t_rel < previous_->t.back() - config_.planner.sampling.dt) {
      Trajectory candidate = slice(*previous_, t_rel, previous_->t.back() + 1.0);
      int checks = 0;
      const CollisionChecker checker(config_.planner.collision, vehicle_);
      if (checker.isCollisionFree(candidate, obstacles, checks)) rest = std::move(candidate);
    }
  }
  const auto retry = [&](bool envelope, double horizon) {
    in.safety_envelope = envelope;
    in.envelope_weight = config_.envelope_retry_weight;
    in.collision_horizon = horizon;
    PlanResult r = planner_->plan(in);
    out.stats.time_ms += r.stats.time_ms;
    if (!r.success) return;
    out.degraded = true;
    res.success = true;
    res.trajectory = std::move(r.trajectory);
  };
  const bool envelope_rejected = res.stats.rejections[static_cast<int>(Rejection::kSafetyDistance)] > 0;
  if (!res.success && !rest && envelope_rejected && config_.planner.safety.enabled) {
    retry(false, std::numeric_limits<double>::infinity());
  }
  if (!res.success && !rest && config_.degraded_horizon > 0.0) retry(false, config_.degraded_horizon);

  if (res.success) {
    out.success = true;
    if (config_.smoother.enabled) {
      out.smoothed = smoothTrajectory(res.trajectory, in, config_.planner, vehicle_, config_.smoother);
    }
    if (prefix) {
      out.trajectory = std::move(*prefix);
      append(out.trajectory, res.trajectory, latency);
      out.trajectory.end = res.trajectory.end;
      out.trajectory.lane = res.trajectory.lane;
      out.trajectory.cost = res.trajectory.cost;
      out.trajectory.idx = res.trajectory.idx;
    } else {
      out.trajectory = std::move(res.trajectory);
    }
    previous_ = out.trajectory;
    previous_t0_ = t_now;
    previous_emergency_ = false;
  } else if (handover) {
    out = std::move(*handover);
    previous_ = out.trajectory;
    previous_t0_ = t_now;
    previous_emergency_ = false;
  } else if (rest) {
    out.trajectory = std::move(*rest);
    out.fallback = true;
  } else {
    out.trajectory = emergencyStop(out.start, ref);
    out.emergency = true;
    previous_ = out.trajectory;
    previous_t0_ = t_now;
    previous_emergency_ = true;
  }
  return out;
}

}  // namespace fiss_planner
