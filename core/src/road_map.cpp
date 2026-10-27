// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/map/road_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "fiss_planner/common/math.hpp"

namespace fiss_planner {

double polylineLength(const std::vector<Vec2>& line) {
  double len = 0.0;
  for (size_t i = 1; i < line.size(); ++i) {
    len += std::hypot(line[i].x - line[i - 1].x, line[i].y - line[i - 1].y);
  }
  return len;
}

double MapLane::length() const { return polylineLength(centerline); }

PolylineProjection projectOnPolyline(const std::vector<Vec2>& line, double x, double y) {
  PolylineProjection best;
  double s0 = 0.0;
  for (size_t i = 1; i < line.size(); ++i) {
    const double ax = line[i - 1].x, ay = line[i - 1].y;
    const double dx = line[i].x - ax, dy = line[i].y - ay;
    const double len2 = dx * dx + dy * dy;
    const double len = std::sqrt(len2);
    if (len2 < 1e-12) continue;
    const double u = std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0);
    const double dist = std::hypot(x - (ax + u * dx), y - (ay + u * dy));
    if (dist < best.distance) {
      best.distance = dist;
      best.s = s0 + u * len;
      best.heading = std::atan2(dy, dx);
    }
    s0 += len;
  }
  return best;
}

Vec2 polylinePoint(const std::vector<Vec2>& line, double s, double* heading) {
  if (line.empty()) return {};
  double s0 = 0.0;
  for (size_t i = 1; i < line.size(); ++i) {
    const double dx = line[i].x - line[i - 1].x, dy = line[i].y - line[i - 1].y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) continue;
    if (s <= s0 + len || i + 1 == line.size()) {
      const double u = std::clamp((s - s0) / len, 0.0, 1.0);
      if (heading) *heading = std::atan2(dy, dx);
      return {line[i - 1].x + u * dx, line[i - 1].y + u * dy};
    }
    s0 += len;
  }
  if (heading) *heading = 0.0;
  return line.back();
}

void RoadMap::addLane(MapLane lane) {
  if (lane.centerline.size() < 2 || lane.left.size() < 2 || lane.right.size() < 2) {
    throw std::invalid_argument("RoadMap::addLane: lane " + std::to_string(lane.id) +
                                " needs centerline and boundaries with >= 2 points");
  }
  const int id = lane.id;
  lanes_[id] = std::move(lane);
}

bool RoadMap::setStopLineActive(int id, bool active, bool yellow) {
  for (auto& sl : stop_lines_) {
    if (sl.id == id) {
      sl.active = active;
      sl.yellow = active && yellow;
      return true;
    }
  }
  return false;
}

void RoadMap::finalize() {
  bool has_predecessors = false;
  for (const auto& [id, lane] : lanes_) has_predecessors |= !lane.predecessors.empty();
  for (auto& [id, lane] : lanes_) {
    const auto drop_missing = [this](std::vector<int>& ids) {
      ids.erase(std::remove_if(ids.begin(), ids.end(), [this](int i) { return !lanes_.count(i); }),
                ids.end());
    };
    drop_missing(lane.successors);
    drop_missing(lane.predecessors);
    if (lane.left_neighbor >= 0 && !lanes_.count(lane.left_neighbor)) lane.left_neighbor = -1;
    if (lane.right_neighbor >= 0 && !lanes_.count(lane.right_neighbor)) lane.right_neighbor = -1;
    if (lane.left_opposite >= 0 && !lanes_.count(lane.left_opposite)) lane.left_opposite = -1;
    if (lane.left_neighbor < 0) lane.can_change_left = false;
    if (lane.right_neighbor < 0) lane.can_change_right = false;
  }
  if (!has_predecessors) {
    for (const auto& [id, lane] : lanes_) {
      for (int succ : lane.successors) lanes_[succ].predecessors.push_back(id);
    }
  }
}

const MapLane* RoadMap::lane(int id) const {
  const auto it = lanes_.find(id);
  return it == lanes_.end() ? nullptr : &it->second;
}

namespace {
bool insideLane(const MapLane& lane, double x, double y) {
  // Ray casting over the polygon left boundary + reversed right boundary.
  std::vector<Vec2> poly(lane.left);
  poly.insert(poly.end(), lane.right.rbegin(), lane.right.rend());
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto& a = poly[i];
    const auto& b = poly[j];
    if (((a.y > y) != (b.y > y)) && (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)) inside = !inside;
  }
  return inside;
}
}  // namespace

int RoadMap::locate(double x, double y, double yaw, const std::unordered_set<int>* preferred,
                    double max_dist) const {
  int best = -1;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto& [id, lane] : lanes_) {
    const auto proj = projectOnPolyline(lane.centerline, x, y);
    if (proj.distance > max_dist + 10.0) continue;
    if (std::abs(normalizeAngle(yaw - proj.heading)) > kPi / 2) continue;
    // Containment dominates, then preference, then distance to the centerline.
    double score = proj.distance;
    if (!insideLane(lane, x, y)) {
      if (proj.distance > max_dist) continue;
      score += 1e3;
    }
    if (preferred && !preferred->count(id)) score += 1e2;
    if (score < best_score) {
      best_score = score;
      best = id;
    }
  }
  return best;
}

}  // namespace fiss_planner
