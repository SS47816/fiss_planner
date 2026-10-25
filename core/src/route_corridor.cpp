// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/map/route_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include "fiss_planner/common/math.hpp"

namespace fiss_planner {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Lateral offsets of a boundary polyline, sampled at the given stations.
/// Stations outside [s_from, s_to] are left untouched.
void sampleBoundary(const ReferenceLine& ref, const std::vector<Vec2>& line, double s_hint, double s_from,
                    double s_to, const std::vector<double>& stations, std::vector<double>& out) {
  // Densify first: map polylines can have vertices tens of meters apart, and
  // the boundary is straight between them while the reference line is smooth,
  // so interpolating projected vertices alone misplaces the edge.
  constexpr double kMaxSpacing = 1.0;
  std::vector<std::pair<double, double>> sd;
  sd.reserve(line.size());
  double hint = s_hint;
  const auto add = [&](double x, double y) {
    const auto pr = ref.project(x, y, hint);
    hint = pr.s;
    sd.emplace_back(pr.s, pr.d);
  };
  for (size_t i = 0; i < line.size(); ++i) {
    if (i > 0) {
      const double dx = line[i].x - line[i - 1].x, dy = line[i].y - line[i - 1].y;
      const int n = static_cast<int>(std::ceil(std::hypot(dx, dy) / kMaxSpacing));
      for (int k = 1; k < n; ++k) {
        const double u = static_cast<double>(k) / n;
        add(line[i - 1].x + u * dx, line[i - 1].y + u * dy);
      }
    }
    add(line[i].x, line[i].y);
  }
  std::sort(sd.begin(), sd.end());
  if (sd.empty()) return;
  for (size_t k = 0; k < stations.size(); ++k) {
    const double s = stations[k];
    if (s < s_from - 1e-6 || s > s_to + 1e-6) continue;
    if (s <= sd.front().first) {
      out[k] = sd.front().second;
    } else if (s >= sd.back().first) {
      out[k] = sd.back().second;
    } else {
      const auto it = std::lower_bound(sd.begin(), sd.end(), std::make_pair(s, -1e300));
      const auto& b = *it;
      const auto& a = *(it - 1);
      const double w = (b.first - a.first) > 1e-9 ? (s - a.first) / (b.first - a.first) : 0.0;
      out[k] = a.second + w * (b.second - a.second);
    }
  }
}

/// Fill NaN gaps of at most `max_gap` stations by linear interpolation.
void fillSmallGaps(std::vector<double>& v, size_t max_gap) {
  size_t i = 0;
  while (i < v.size()) {
    if (!std::isnan(v[i])) {
      ++i;
      continue;
    }
    size_t j = i;
    while (j < v.size() && std::isnan(v[j])) ++j;
    const bool has_left = i > 0, has_right = j < v.size();
    if (j - i <= max_gap && (has_left || has_right)) {
      for (size_t k = i; k < j; ++k) {
        if (has_left && has_right) {
          const double w = static_cast<double>(k - i + 1) / static_cast<double>(j - i + 1);
          v[k] = v[i - 1] + w * (v[j] - v[i - 1]);
        } else {
          v[k] = has_left ? v[i - 1] : v[j];
        }
      }
    }
    i = j;
  }
}

int pick(const std::vector<int>& candidates, const std::unordered_set<int>& route) {
  for (int c : candidates) {
    if (route.count(c)) return c;
  }
  return candidates.empty() ? -1 : candidates.front();
}

}  // namespace

std::optional<RouteCorridor> buildRouteCorridor(const RoadMap& map, const std::vector<int>& route,
                                                const EgoState& ego, const RouteCorridorConfig& cfg,
                                                int previous_ego_lane) {
  const std::unordered_set<int> route_set(route.begin(), route.end());
  std::unordered_set<int> preferred = route_set;
  if (previous_ego_lane >= 0) preferred.insert(previous_ego_lane);
  const int ego_lane = map.locate(ego.x, ego.y, ego.yaw, &preferred);
  if (ego_lane < 0) return std::nullopt;

  // --- Ego lane chain ---------------------------------------------------------
  const MapLane* l0 = map.lane(ego_lane);
  const double pos0 = projectOnPolyline(l0->centerline, ego.x, ego.y).s;
  std::vector<int> chain = {ego_lane};
  std::unordered_set<int> visited = {ego_lane};
  double behind = pos0;
  while (behind < cfg.lookbehind) {
    const int p = pick(map.lane(chain.front())->predecessors, route_set);
    if (p < 0 || visited.count(p)) break;
    chain.insert(chain.begin(), p);
    visited.insert(p);
    behind += map.lane(p)->length();
  }
  const size_t i0 = static_cast<size_t>(std::find(chain.begin(), chain.end(), ego_lane) - chain.begin());
  double ahead = l0->length() - pos0;
  while (ahead < cfg.lookahead) {
    const int s = pick(map.lane(chain.back())->successors, route_set);
    if (s < 0 || visited.count(s)) break;
    chain.push_back(s);
    visited.insert(s);
    ahead += map.lane(s)->length();
  }

  // --- Reference line ---------------------------------------------------------
  std::vector<double> xs, ys;
  for (int id : chain) {
    for (const auto& p : map.lane(id)->centerline) {
      if (!xs.empty() && std::hypot(p.x - xs.back(), p.y - ys.back()) < 0.05) continue;
      xs.push_back(p.x);
      ys.push_back(p.y);
    }
  }
  RouteCorridor out;
  try {
    out.reference = ReferenceLine(xs, ys);
  } catch (const std::invalid_argument&) {
    return std::nullopt;
  }
  out.ego_chain = chain;
  out.ego_lane = ego_lane;
  out.speed_limit = l0->speed_limit;

  // Arc length of each chain lane's start/end on the reference.
  std::vector<double> lane_s0(chain.size()), lane_s1(chain.size());
  double hint = 0.0;
  for (size_t i = 0; i < chain.size(); ++i) {
    const auto& c = map.lane(chain[i])->centerline;
    lane_s0[i] = out.reference.project(c.front().x, c.front().y, hint).s;
    lane_s1[i] = out.reference.project(c.back().x, c.back().y, lane_s0[i] + map.lane(chain[i])->length()).s;
    hint = lane_s1[i];
  }
  out.ego_s = out.reference.project(ego.x, ego.y, lane_s0[i0] + pos0).s;

  const double length = out.reference.length();
  const auto stations =
      linspace(0.0, length, std::max(2, static_cast<int>(std::ceil(length / cfg.station_ds)) + 1));
  Corridor corridor(stations);

  // --- Lane bands -------------------------------------------------------------
  struct Band {
    int offset;
    std::vector<int> lanes;  // per chain index, -1 if absent
    std::vector<double> left, right;
    bool allowed = true;
  };
  std::vector<Band> bands;
  bands.reserve(1 + 2 * static_cast<size_t>(std::max(0, cfg.max_adjacent_lanes)));
  const auto sample_band = [&](Band& b) {
    b.left.assign(stations.size(), kNaN);
    b.right.assign(stations.size(), kNaN);
    for (size_t i = 0; i < chain.size(); ++i) {
      if (b.lanes[i] < 0) continue;
      const MapLane* ln = map.lane(b.lanes[i]);
      const auto& c = ln->centerline;
      const double sa = out.reference.project(c.front().x, c.front().y, lane_s0[i]).s;
      const double sb = out.reference.project(c.back().x, c.back().y, lane_s1[i]).s;
      sampleBoundary(out.reference, ln->left, lane_s0[i], std::min(sa, sb), std::max(sa, sb), stations,
                     b.left);
      sampleBoundary(out.reference, ln->right, lane_s0[i], std::min(sa, sb), std::max(sa, sb), stations,
                     b.right);
    }
    const size_t gap = static_cast<size_t>(std::ceil(3.0 / cfg.station_ds));
    fillSmallGaps(b.left, gap);
    fillSmallGaps(b.right, gap);
  };

  Band ego_band{0, chain, {}, {}, true};
  sample_band(ego_band);
  fillSmallGaps(ego_band.left, stations.size());
  fillSmallGaps(ego_band.right, stations.size());
  bands.push_back(ego_band);

  for (int side : {+1, -1}) {
    const Band* inner = &bands.front();
    for (int k = 1; k <= cfg.max_adjacent_lanes; ++k) {
      Band b{side * k, std::vector<int>(chain.size(), -1), {}, {}, false};
      bool any = false;
      for (size_t i = 0; i < chain.size(); ++i) {
        if (inner->lanes[i] < 0) continue;
        const MapLane* in = map.lane(inner->lanes[i]);
        b.lanes[i] = side > 0 ? in->left_neighbor : in->right_neighbor;
        any |= b.lanes[i] >= 0;
      }
      if (!any) break;
      // Accessibility at the first chain index (from the ego onwards) where the band exists.
      for (size_t i = i0; i < chain.size(); ++i) {
        if (b.lanes[i] < 0 || inner->lanes[i] < 0) continue;
        const MapLane* in = map.lane(inner->lanes[i]);
        // A route through the neighbour implies the change is legal (routing
        // graphs only contain permitted changes), even if markings disagree.
        const bool permitted = side > 0 ? in->can_change_left : in->can_change_right;
        const bool on_route = route_set.count(b.lanes[i]) > 0;
        b.allowed = inner->allowed && (permitted || on_route);
        break;
      }
      sample_band(b);
      // Where the lane does not exist, collapse it onto the shared edge.
      for (size_t s = 0; s < stations.size(); ++s) {
        if (std::isnan(b.left[s]) || std::isnan(b.right[s])) {
          const double edge = side > 0 ? inner->left[s] : inner->right[s];
          b.left[s] = b.right[s] = edge;
        }
      }
      bands.push_back(std::move(b));
      inner = &bands.back();
    }
  }

  for (const auto& b : bands) {
    LaneBand lane;
    lane.offset = b.offset;
    lane.change_allowed = b.allowed;
    lane.left = b.left;
    lane.right = b.right;
    lane.center.resize(stations.size());
    for (size_t s = 0; s < stations.size(); ++s) lane.center[s] = 0.5 * (b.left[s] + b.right[s]);

    // Lane id / speed limit at the ego, or where the lane first appears ahead.
    for (size_t i = i0; i < chain.size(); ++i) {
      if (b.lanes[i] >= 0) {
        lane.lane_id = b.lanes[i];
        lane.speed_limit = map.lane(b.lanes[i])->speed_limit;
        break;
      }
    }
    // Route urgency: distance until the band leaves the route (or ends).
    double leave = std::numeric_limits<double>::infinity();
    for (size_t i = i0; i < chain.size(); ++i) {
      const bool gone = b.lanes[i] < 0 || (!route_set.empty() && !route_set.count(b.lanes[i]));
      if (gone) {
        leave = std::max(0.0, lane_s0[i] - out.ego_s);
        break;
      }
    }
    if (std::isfinite(leave) && cfg.urgency_distance > 0.0) {
      lane.extra_cost = cfg.w_route_urgency * std::max(0.0, 1.0 - leave / cfg.urgency_distance);
    }
    corridor.addLane(std::move(lane));
  }
  out.corridor = std::move(corridor);

  // --- Stop lines -------------------------------------------------------------
  std::unordered_set<int> band_lanes;
  for (const auto& b : bands) {
    for (size_t i = i0; i < chain.size(); ++i) {
      if (b.lanes[i] >= 0) band_lanes.insert(b.lanes[i]);
    }
  }
  for (const auto& sl : map.stopLines()) {
    if (!sl.active || !band_lanes.count(sl.lane_id)) continue;
    const MapLane* ln = map.lane(sl.lane_id);
    double heading = 0.0;
    const Vec2 p = polylinePoint(ln->centerline, sl.s, &heading);
    if (out.reference.project(p.x, p.y, out.ego_s).s <= out.ego_s) continue;  // already passed
    const double width =
        projectOnPolyline(ln->left, p.x, p.y).distance + projectOnPolyline(ln->right, p.x, p.y).distance;
    Obstacle ob;
    ob.id = -1000 - sl.id;
    ob.length = 1.0;
    ob.width = std::max(width, 1.0);
    ob.is_static = true;
    ob.prediction.push_back({0.0, p.x, p.y, heading, 0.0});
    out.stop_lines.push_back(std::move(ob));
  }
  return out;
}

}  // namespace fiss_planner
