// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/planning/corridor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "frenet_planner/common/math.hpp"

namespace frenet_planner {

namespace {
std::vector<double> makeStations(double s_start, double s_end, double ds) {
  if (!(s_end > s_start) || !(ds > 0.0)) throw std::invalid_argument("Corridor: invalid station range");
  return linspace(s_start, s_end, static_cast<int>(std::ceil((s_end - s_start) / ds)) + 1);
}
}  // namespace

Corridor Corridor::uniform(double s_start, double s_end, double d_right, double d_left, double ds) {
  if (!(d_left > d_right)) throw std::invalid_argument("Corridor::uniform: d_left must exceed d_right");
  Corridor c(makeStations(s_start, s_end, ds));
  LaneBand lane;
  const size_t n = c.s_.size();
  lane.center.assign(n, 0.5 * (d_left + d_right));
  lane.left.assign(n, d_left);
  lane.right.assign(n, d_right);
  c.addLane(std::move(lane));
  return c;
}

Corridor Corridor::parallelLanes(double s_start, double s_end, double lane_width, int num_left, int num_right,
                                 double ds) {
  if (!(lane_width > 0.0) || num_left < 0 || num_right < 0) {
    throw std::invalid_argument("Corridor::parallelLanes: invalid lane layout");
  }
  Corridor c(makeStations(s_start, s_end, ds));
  const size_t n = c.s_.size();
  for (int k = -num_right; k <= num_left; ++k) {
    LaneBand lane;
    lane.offset = k;
    lane.center.assign(n, k * lane_width);
    lane.left.assign(n, (k + 0.5) * lane_width);
    lane.right.assign(n, (k - 0.5) * lane_width);
    c.addLane(std::move(lane));
  }
  return c;
}

void Corridor::addLane(LaneBand lane) {
  const size_t n = s_.size();
  if (lane.center.size() != n || lane.left.size() != n || lane.right.size() != n) {
    throw std::invalid_argument("Corridor::addLane: lane arrays must match the station count");
  }
  lanes_.push_back(std::move(lane));
  std::sort(lanes_.begin(), lanes_.end(),
            [](const LaneBand& a, const LaneBand& b) { return a.offset < b.offset; });
}

const LaneBand* Corridor::egoLane() const {
  for (const auto& lane : lanes_) {
    if (lane.offset == 0) return &lane;
  }
  return nullptr;
}

double Corridor::interp(const std::vector<double>& values, double s) const {
  if (s <= s_.front()) return values.front();
  if (s >= s_.back()) return values.back();
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const size_t i = static_cast<size_t>(it - s_.begin());
  const double w = (s - s_[i - 1]) / (s_[i] - s_[i - 1]);
  return values[i - 1] + w * (values[i] - values[i - 1]);
}

std::pair<double, double> Corridor::roadBounds(double s) const {
  double lo = std::numeric_limits<double>::infinity();
  double hi = -std::numeric_limits<double>::infinity();
  for (const auto& lane : lanes_) {
    if (!lane.change_allowed && lane.offset != 0) continue;
    lo = std::min(lo, right(lane, s));
    hi = std::max(hi, left(lane, s));
  }
  return {lo, hi};
}

}  // namespace frenet_planner
