// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/prediction/lane_following.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include "frenet_planner/common/math.hpp"

namespace frenet_planner {

namespace {

/// Successor whose start continues most straight from the end of `lane`.
const MapLane* straightestSuccessor(const RoadMap& map, const MapLane& lane) {
  double end_heading = 0.0;
  polylinePoint(lane.centerline, lane.length(), &end_heading);
  const MapLane* best = nullptr;
  double best_turn = 0.0;
  for (const int id : lane.successors) {
    const MapLane* next = map.lane(id);
    if (next == nullptr || next->centerline.size() < 2) continue;
    double start_heading = 0.0;
    polylinePoint(next->centerline, 0.0, &start_heading);
    const double turn = std::abs(normalizeAngle(start_heading - end_heading));
    if (best == nullptr || turn < best_turn) {
      best = next;
      best_turn = turn;
    }
  }
  return best;
}

}  // namespace

Obstacle predictLaneFollowing(const RoadMap& map, int id, double x, double y, double yaw, double v,
                              double length, double width, double horizon, double dt,
                              const LaneFollowingConfig& cfg) {
  const auto fallback = [&] { return predictConstantVelocity(id, x, y, yaw, v, length, width, horizon, dt); };
  if (v < cfg.min_speed) return fallback();
  const MapLane* lane = map.lane(map.locate(x, y, yaw, nullptr, cfg.max_lane_distance));
  if (lane == nullptr) return fallback();
  const PolylineProjection proj = projectOnPolyline(lane->centerline, x, y);
  if (proj.distance > cfg.max_lane_distance ||
      std::abs(normalizeAngle(yaw - proj.heading)) > cfg.max_heading_diff) {
    return fallback();
  }
  // Signed offset from the centerline (positive to the left).
  double heading = 0.0;
  const Vec2 foot = polylinePoint(lane->centerline, proj.s, &heading);
  const double offset0 = -(x - foot.x) * std::sin(heading) + (y - foot.y) * std::cos(heading);

  Obstacle ob;
  ob.id = id;
  ob.length = length;
  ob.width = width;
  double s = proj.s;  // arc length along the current lane
  double lane_len = lane->length();
  double beyond = 0.0;  // distance driven past a lane without successors
  const int n = static_cast<int>(std::floor(horizon / dt + 1e-9)) + 1;
  for (int k = 0; k < n; ++k) {
    const double t = k * dt;
    if (k > 0 && beyond > 0.0) {
      beyond += v * dt;
    } else if (k > 0) {
      s += v * dt;
      while (s > lane_len) {
        const MapLane* next = straightestSuccessor(map, *lane);
        if (next == nullptr) {
          beyond = s - lane_len;
          s = lane_len;
          break;
        }
        s -= lane_len;
        lane = next;
        lane_len = lane->length();
      }
    }
    Vec2 p = polylinePoint(lane->centerline, s, &heading);
    p.x += beyond * std::cos(heading);  // past a dead end: continue straight
    p.y += beyond * std::sin(heading);
    const double offset = offset0 * std::exp(-t / cfg.lateral_time_constant);
    ob.prediction.push_back(
        {t, p.x - offset * std::sin(heading), p.y + offset * std::cos(heading), heading, v});
  }
  return ob;
}

PredictionMode predictionModeFromString(const std::string& name) {
  std::string n = name;
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return std::tolower(c); });
  if (n == "as_given") return PredictionMode::kAsGiven;
  if (n == "constant_velocity") return PredictionMode::kConstantVelocity;
  if (n == "lane_following") return PredictionMode::kLaneFollowing;
  throw std::invalid_argument("unknown prediction mode '" + name +
                              "' (expected as_given, constant_velocity, lane_following)");
}

std::string toString(PredictionMode mode) {
  switch (mode) {
    case PredictionMode::kAsGiven:
      return "as_given";
    case PredictionMode::kConstantVelocity:
      return "constant_velocity";
    case PredictionMode::kLaneFollowing:
      return "lane_following";
  }
  return "as_given";
}

void completePredictions(std::vector<Obstacle>& obstacles, PredictionMode mode, const RoadMap* map,
                         double horizon, double dt, const LaneFollowingConfig& cfg) {
  if (mode == PredictionMode::kAsGiven) return;
  for (auto& ob : obstacles) {
    if (ob.is_static || ob.prediction.size() != 1) continue;
    const ObstacleState p = ob.prediction.front();
    Obstacle predicted =
        mode == PredictionMode::kLaneFollowing && map != nullptr
            ? predictLaneFollowing(*map, ob.id, p.x, p.y, p.yaw, p.v, ob.length, ob.width, horizon, dt, cfg)
            : predictConstantVelocity(ob.id, p.x, p.y, p.yaw, p.v, ob.length, ob.width, horizon, dt);
    for (auto& st : predicted.prediction) st.t += p.t;  // keep the observation's time offset
    ob.prediction = std::move(predicted.prediction);
  }
}

}  // namespace frenet_planner
