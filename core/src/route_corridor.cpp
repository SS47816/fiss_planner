// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/map/route_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
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

/// The candidate furthest along the route (routes from waypoints can touch a
/// lane just before turning off from its predecessor), else the first one.
int pick(const std::vector<int>& candidates, const std::unordered_map<int, size_t>& route_index) {
  int best = candidates.empty() ? -1 : candidates.front();
  size_t best_index = 0;
  bool on_route = false;
  for (int c : candidates) {
    const auto it = route_index.find(c);
    if (it != route_index.end() && (!on_route || it->second > best_index)) {
      best = c;
      best_index = it->second;
      on_route = true;
    }
  }
  return best;
}

/// Lane changes the route still needs. A route lane needs the lane changes
/// the route makes right after it, until it continues lengthwise (along this
/// stretch of side-by-side lanes); any other lane first moves sideways, over
/// permitted changes, to the route lane that needs the fewest in total.
class RouteProgress {
 public:
  RouteProgress(const RoadMap& map, const std::vector<int>& route) : map_(map) {
    int changes = 0;
    for (size_t k = route.size(); k-- > 0;) {
      const MapLane* lane = map.lane(route[k]);
      if (!lane) continue;
      if (k + 1 < route.size() &&
          (lane->left_neighbor == route[k + 1] || lane->right_neighbor == route[k + 1])) {
        ++changes;
        route_changes_.insert(key(route[k], route[k + 1]));
      } else {
        changes = 0;  // the route continues lengthwise (or ends) here
      }
      const auto it = changes_.find(route[k]);
      changes_[route[k]] = it == changes_.end() ? changes : std::min(it->second, changes);
    }
  }

  bool empty() const { return changes_.empty(); }
  bool onRoute(int lane) const { return changes_.count(lane) > 0; }

  /// Lane changes needed from `lane` before the route leaves its stretch of
  /// road; -1 if no route lane can be reached sideways.
  int changesNeeded(int lane) const {
    const auto cached = cache_.find(lane);
    if (cached != cache_.end()) return cached->second;
    int best = -1;
    walk(lane, [&](int id, int steps) {
      const auto it = changes_.find(id);
      if (it != changes_.end() && (best < 0 || steps + it->second < best)) best = steps + it->second;
    });
    cache_[lane] = best;
    return best;
  }

  /// Lane changes from `lane` sideways to `target`; -1 if not reachable.
  int stepsTo(int lane, int target) const {
    int best = -1;
    walk(lane, [&](int id, int steps) {
      if (id == target && (best < 0 || steps < best)) best = steps;
    });
    return best;
  }

 private:
  static constexpr int kMaxSteps = 8;
  static int64_t key(int from, int to) {
    return (static_cast<int64_t>(from) << 32) ^ static_cast<uint32_t>(to);
  }

  /// Visit `lane` and the lanes reachable from it sideways over permitted
  /// changes (or the route's own) with the number of changes.
  template <typename F>
  void walk(int lane, F&& visit) const {
    visit(lane, 0);
    for (const bool left : {true, false}) {
      int cur = lane;
      for (int steps = 1; steps <= kMaxSteps; ++steps) {
        const MapLane* l = map_.lane(cur);
        const int next = l ? (left ? l->left_neighbor : l->right_neighbor) : -1;
        if (next < 0) break;
        const bool permitted = left ? l->can_change_left : l->can_change_right;
        if (!permitted && !route_changes_.count(key(cur, next))) break;
        cur = next;
        visit(cur, steps);
      }
    }
  }

  const RoadMap& map_;
  std::unordered_map<int, int> changes_;       ///< route lane -> lane changes of the route right after it
  std::unordered_set<int64_t> route_changes_;  ///< (from, to) lane changes of the route
  mutable std::unordered_map<int, int> cache_;
};

}  // namespace

std::optional<RouteCorridor> buildRouteCorridor(const RoadMap& map, const std::vector<int>& route,
                                                const EgoState& ego, const RouteCorridorConfig& cfg,
                                                int previous_ego_lane, const std::optional<Vec2>& goal) {
  const std::unordered_set<int> route_set(route.begin(), route.end());
  std::unordered_map<int, size_t> route_index;  // last position of each lane in the route
  for (size_t k = 0; k < route.size(); ++k) route_index[route[k]] = k;
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
    const int p = pick(map.lane(chain.front())->predecessors, route_index);
    if (p < 0 || visited.count(p)) break;
    chain.insert(chain.begin(), p);
    visited.insert(p);
    behind += map.lane(p)->length();
  }
  const size_t i0 = static_cast<size_t>(std::find(chain.begin(), chain.end(), ego_lane) - chain.begin());
  double ahead = l0->length() - pos0;
  while (ahead < cfg.lookahead) {
    const int s = pick(map.lane(chain.back())->successors, route_index);
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
  if (cfg.smoothing_length > 0.0) smoothPolyline(xs, ys, cfg.smoothing_length, cfg.smoothing_tolerance);
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

  // --- Route urgency ----------------------------------------------------------
  const RouteProgress progress(map, route);
  const double inf = std::numeric_limits<double>::infinity();
  const double ramp = std::max(cfg.urgency_distance, cfg.urgency_time * std::max(0.0, ego.v));
  // Lane changes can be due beyond the reference: for urgency only, follow
  // the chain further (along the route where it goes on).
  std::vector<int> ext = chain;
  std::vector<double> ext_s0 = lane_s0, ext_s1 = lane_s1;
  if (!progress.empty()) {
    constexpr double kUrgencyHorizon = 4.0;  // in ramp lengths: beyond this no urgency arises (n <= 4)
    while (ext_s1.back() < out.ego_s + kUrgencyHorizon * ramp) {
      const int next = pick(map.lane(ext.back())->successors, route_index);
      if (next < 0 || visited.count(next)) break;
      visited.insert(next);
      ext.push_back(next);
      ext_s0.push_back(ext_s1.back());
      ext_s1.push_back(ext_s1.back() + map.lane(next)->length());
    }
  }
  // Lane of a band level with chain index i (also beyond the reference).
  const auto band_lane = [&](const Band& b, size_t i) {
    if (i < chain.size()) return b.lanes[i];
    int id = ext[i];
    for (int k = 0; k < std::abs(b.offset) && id >= 0; ++k) {
      const MapLane* l = map.lane(id);
      id = b.offset > 0 ? l->left_neighbor : l->right_neighbor;
    }
    return id;
  };
  // Chain index level with the goal, and the distance to it.
  size_t goal_index = ext.size();
  double goal_dist = inf;
  if (goal && !progress.empty()) {
    constexpr double kGoalLateralRange = 20.0;  // [m] the goal must be beside the ego's lane chain
    const auto pg = out.reference.project(goal->x, goal->y);
    if (pg.s <= out.reference.length()) {
      if (pg.s > out.ego_s && std::abs(pg.d) < kGoalLateralRange) {
        for (size_t i = i0; i < chain.size() && goal_index == ext.size(); ++i) {
          if (pg.s <= lane_s1[i] + 1e-6) goal_index = i;
        }
        goal_dist = pg.s - out.ego_s;
      }
    } else {
      // Beyond the reference: where the chain runs beside the destination lane.
      const int dest = route.back();
      const auto beside = [&](int id) {
        for (const bool left : {true, false}) {
          for (int cur = id, k = 0; cur >= 0 && k <= 8; ++k) {
            if (cur == dest) return true;
            const MapLane* l = map.lane(cur);
            cur = left ? l->left_neighbor : l->right_neighbor;
          }
        }
        return false;
      };
      for (size_t i = chain.size(); i < ext.size() && goal_index == ext.size(); ++i) {
        if (!beside(ext[i])) continue;
        goal_index = i;
        goal_dist = ext_s0[i] + projectOnPolyline(map.lane(dest)->centerline, goal->x, goal->y).s - out.ego_s;
      }
    }
  }
  // Lane changes due along each band: (changes, distance) wherever the band
  // can no longer go on as it is. The most urgent of them sets its cost.
  struct Urgency {
    bool off_route = false;  ///< the band's lane beside the ego is not a route lane
    bool reachable = true;   ///< the route can be reached from it
    double urgency = 0.0;
    double cost = 0.0;
  };
  const auto due = [&](Urgency& u, int changes, double dist) {
    if (changes <= 0 || !(ramp > 0.0)) return;
    // Zero while the changes fit comfortably, w * n with half the ramp
    // distance per change left, then steeply growing (capped).
    constexpr double kMaxFactor = 10.0;
    const double n = changes;
    const double urg = std::clamp(1.0 - std::max(0.0, dist) / (n * ramp), 0.0, 1.0);
    const double factor = urg < 1.0 ? std::min(urg / (1.0 - urg), kMaxFactor) : kMaxFactor;
    u.urgency = std::max(u.urgency, urg);
    u.cost = std::max(u.cost, cfg.w_route_urgency * n * factor);
  };
  std::vector<Urgency> urgency(bands.size());
  int most_changes = 0;
  for (size_t j = 0; j < bands.size(); ++j) {
    const Band& b = bands[j];
    size_t first = i0;
    while (first < chain.size() && b.lanes[first] < 0) ++first;
    if (first == chain.size()) continue;
    Urgency& u = urgency[j];
    if (progress.empty()) {
      // Without a route, a band only has to be left where it ends.
      bool ended = false;
      for (size_t i = first; i < chain.size() && !ended; ++i) {
        if (b.lanes[i] < 0) {
          due(u, 1, lane_s0[i] - out.ego_s);
          ended = true;
        }
      }
      const MapLane* last = map.lane(b.lanes.back());
      if (!ended && last && last->successors.empty()) due(u, 1, lane_s1.back() - out.ego_s);
      continue;
    }
    int changes = progress.changesNeeded(b.lanes[first]);
    u.off_route = !progress.onRoute(b.lanes[first]);
    u.reachable = changes >= 0;
    if (!u.reachable) continue;
    most_changes = std::max(most_changes, changes);
    bool on_route = !u.off_route;
    bool ended = false;
    for (size_t i = first; i < ext.size() && !ended; ++i) {
      const int lane = band_lane(b, i);
      if (i > first) {
        const int next = lane < 0 ? -1 : progress.changesNeeded(lane);
        if (next < 0) {
          // The band ends or loses the route: off the route it has to be left,
          // on it only the route's own changes remain (none past its end).
          due(u, on_route ? changes : std::max(1, changes), ext_s0[i] - out.ego_s);
          ended = true;
          continue;
        }
        // A route lane is left where the route leaves it; off the route,
        // the changes can still be made further on.
        const bool now_on = progress.onRoute(lane);
        if (on_route && !now_on) due(u, changes, ext_s0[i] - out.ego_s);
        on_route = now_on;
        changes = next;
      }
      if (i == goal_index) due(u, progress.stepsTo(lane, route.back()), goal_dist);
    }
    const MapLane* last = map.lane(band_lane(b, ext.size() - 1));
    if (!ended && last && last->successors.empty()) due(u, changes, ext_s1.back() - out.ego_s);  // dead end
  }

  for (size_t j = 0; j < bands.size(); ++j) {
    const Band& b = bands[j];
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
    Urgency u = urgency[j];
    if (!u.reachable) {  // cannot reach the route: worse than any band that can
      u.urgency = 1.0;
      due(u, most_changes + 1, 0.0);
    }
    lane.urgency = u.urgency;
    lane.extra_cost = u.cost + (u.off_route ? cfg.w_route_urgency : 0.0);
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
    const double s_ref = out.reference.project(p.x, p.y, out.ego_s).s;
    if (s_ref <= out.ego_s) continue;  // already passed
    const double width =
        projectOnPolyline(ln->left, p.x, p.y).distance + projectOnPolyline(ln->right, p.x, p.y).distance;
    Obstacle ob;
    ob.id = -1000 - sl.id;
    ob.length = 1.0;
    ob.width = std::max(width, 1.0);
    ob.is_static = true;
    // The planner stops a gap short of obstacles; the offset brings the front to the line.
    const double off = cfg.stop_line_offset;
    ob.prediction.push_back(
        {0.0, p.x + off * std::cos(heading), p.y + off * std::sin(heading), heading, 0.0});
    out.stop_lines.push_back(std::move(ob));
    out.stop_line_info.push_back({sl.id, s_ref, sl.yellow, sl.stop_sign});
  }

  // --- Junctions ----------------------------------------------------------------
  for (size_t i = 0; i < chain.size(); ++i) {
    if (!map.lane(chain[i])->junction) continue;
    const double s_in = std::min(lane_s0[i], lane_s1[i]), s_out = std::max(lane_s0[i], lane_s1[i]);
    if (!out.junctions.empty() && s_in <= out.junctions.back().second + 1.0) {
      out.junctions.back().second = std::max(out.junctions.back().second, s_out);
    } else {
      out.junctions.emplace_back(s_in, s_out);
    }
  }
  return out;
}

}  // namespace fiss_planner
