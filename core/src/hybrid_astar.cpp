// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/freespace/hybrid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace fiss_planner {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kPathSpacing = 0.2;   // [m] between the poses of a planned path
constexpr double kGoalDistance = 0.5;  // [m] a search node this close to the goal ...
constexpr double kGoalYaw = 0.1;       // [rad] ... and this well aligned reaches it
constexpr double kShotRange = 25.0;    // [m] try the Dubins shot to the goal within this distance

double wrapAngle(double a) {
  a = std::fmod(a + M_PI, kTwoPi);
  if (a < 0.0) a += kTwoPi;
  return a - M_PI;
}

double mod2pi(double a) {
  a = std::fmod(a, kTwoPi);
  return a < 0.0 ? a + kTwoPi : a;
}

// The search runs on the rear axle, which follows the kinematic bicycle
// exactly; the grid and the output use the vehicle center.
FreeSpacePose toRear(const FreeSpacePose& p, const VehicleParams& v) {
  return {p.x - v.rear_axle_to_center * std::cos(p.yaw), p.y - v.rear_axle_to_center * std::sin(p.yaw), p.yaw,
          p.reverse};
}

FreeSpacePose toCenter(const FreeSpacePose& p, const VehicleParams& v) {
  return {p.x + v.rear_axle_to_center * std::cos(p.yaw), p.y + v.rear_axle_to_center * std::sin(p.yaw), p.yaw,
          p.reverse};
}

// Pose after driving `ds` (negative = backwards) at curvature `k`.
FreeSpacePose drive(const FreeSpacePose& p, double k, double ds) {
  FreeSpacePose q = p;
  if (std::abs(k) < 1e-9) {
    q.x += ds * std::cos(p.yaw);
    q.y += ds * std::sin(p.yaw);
  } else {
    q.yaw = p.yaw + k * ds;
    q.x += (std::sin(q.yaw) - std::sin(p.yaw)) / k;
    q.y -= (std::cos(q.yaw) - std::cos(p.yaw)) / k;
  }
  q.yaw = wrapAngle(q.yaw);
  q.reverse = ds < 0.0;
  return q;
}

// The vehicle box covered twice: by a few large circles along its length
// (fast, but they reach well past its ends), and by small circles on a fine
// lattice that reach at most kFineExcess past it.
constexpr double kFineExcess = 0.05;  // [m]

struct Footprint {
  std::vector<double> offsets;  // coarse circle centers along the heading
  double radius = 0.0;
  std::vector<Vec2> fine;  // fine circle centers (along, across)
  double fine_radius = 0.0;
};

Footprint footprint(const VehicleParams& v) {
  Footprint f;
  const int n = std::max(1, static_cast<int>(std::ceil(v.length / (0.5 * v.width))));
  f.radius = std::hypot(0.5 * v.length / n, 0.5 * v.width);
  for (int k = 0; k < n; ++k) f.offsets.push_back(-0.5 * v.length + (k + 0.5) * v.length / n);
  // Squares of side h tile the box; the circle around each square reaches
  // h (sqrt(2) - 1) / 2 past its sides.
  const double h = 2.0 * kFineExcess / (M_SQRT2 - 1.0);
  const int nl = std::max(1, static_cast<int>(std::ceil(v.length / h)));
  const int nw = std::max(1, static_cast<int>(std::ceil(v.width / h)));
  const double hl = v.length / nl, hw = v.width / nw;
  f.fine_radius = 0.5 * std::hypot(hl, hw);
  for (int i = 0; i < nl; ++i) {
    for (int j = 0; j < nw; ++j)
      f.fine.push_back({-0.5 * v.length + (i + 0.5) * hl, -0.5 * v.width + (j + 0.5) * hw});
  }
  return f;
}

// Clearance of the box (to within kFineExcess).
double footprintClearance(const OccupancyGrid& grid, const FreeSpacePose& center, const Footprint& f) {
  const double c = std::cos(center.yaw), s = std::sin(center.yaw);
  double clear = kInf;
  for (const Vec2& p : f.fine) {
    clear = std::min(
        clear, grid.clearance(center.x + p.x * c - p.y * s, center.y + p.x * s + p.y * c) - f.fine_radius);
  }
  return clear;
}

bool footprintFree(const OccupancyGrid& grid, const FreeSpacePose& center, const Footprint& f,
                   double margin) {
  const double c = std::cos(center.yaw), s = std::sin(center.yaw);
  bool coarse = true;
  for (double o : f.offsets) {
    if (grid.clearance(center.x + o * c, center.y + o * s) < f.radius + margin) {
      coarse = false;
      break;
    }
  }
  if (coarse) return true;
  for (const Vec2& p : f.fine) {
    if (grid.clearance(center.x + p.x * c - p.y * s, center.y + p.x * s + p.y * c) < f.fine_radius + margin)
      return false;
  }
  return true;
}

// Felzenszwalb & Huttenlocher squared distance transform of a sampled function.
void distanceTransform1d(const std::vector<double>& f, std::vector<double>& d) {
  const int n = static_cast<int>(f.size());
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    if (f[q] == kInf) continue;
    if (f[v[k]] == kInf) {
      v[k] = q;
      continue;
    }
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (k > 0 && s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  d.assign(n, kInf);
  if (f[v[0]] == kInf) return;
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

// ---------------------------------------------------------------- grid

OccupancyGrid::OccupancyGrid(double cx, double cy, double size, double resolution)
    : x0_(std::floor((cx - 0.5 * size) / resolution) * resolution),
      y0_(std::floor((cy - 0.5 * size) / resolution) * resolution),
      res_(resolution),
      n_(std::max(1, static_cast<int>(std::ceil(size / resolution)))),
      cells_(static_cast<size_t>(n_) * n_, 1) {}

void OccupancyGrid::cell(double x, double y, int& i, int& j) const {
  i = static_cast<int>(std::floor((x - x0_) / res_));
  j = static_cast<int>(std::floor((y - y0_) / res_));
}

Vec2 OccupancyGrid::center(int i, int j) const { return {x0_ + (i + 0.5) * res_, y0_ + (j + 0.5) * res_}; }

bool OccupancyGrid::inside(double x, double y) const {
  return x >= x0_ && y >= y0_ && x < x0_ + n_ * res_ && y < y0_ + n_ * res_;
}

void OccupancyGrid::markFree(const std::vector<Vec2>& polygon) {
  const size_t m = polygon.size();
  if (m < 3) return;
  std::vector<double> xs;
  for (int j = 0; j < n_; ++j) {
    const double y = y0_ + (j + 0.5) * res_;
    xs.clear();
    for (size_t e = 0; e < m; ++e) {
      const Vec2& a = polygon[e];
      const Vec2& b = polygon[(e + 1) % m];
      if ((a.y <= y) == (b.y <= y)) continue;
      xs.push_back(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
    }
    std::sort(xs.begin(), xs.end());
    for (size_t k = 0; k + 1 < xs.size(); k += 2) {
      const int i0 = std::max(0, static_cast<int>(std::ceil((xs[k] - x0_) / res_ - 0.5)));
      const int i1 = std::min(n_ - 1, static_cast<int>(std::floor((xs[k + 1] - x0_) / res_ - 0.5)));
      for (int i = i0; i <= i1; ++i) cells_[index(i, j)] = 0;
    }
  }
  clearance_.clear();
}

void OccupancyGrid::markOccupied(const Box2d& box, double inflation) {
  const double hl = 0.5 * box.length + inflation, hw = 0.5 * box.width + inflation;
  const double c = std::cos(box.yaw), s = std::sin(box.yaw);
  const double r = std::hypot(hl, hw);
  int i0, j0, i1, j1;
  cell(box.cx - r, box.cy - r, i0, j0);
  cell(box.cx + r, box.cy + r, i1, j1);
  for (int j = std::max(0, j0); j <= std::min(n_ - 1, j1); ++j) {
    for (int i = std::max(0, i0); i <= std::min(n_ - 1, i1); ++i) {
      const Vec2 p = center(i, j);
      const double dx = p.x - box.cx, dy = p.y - box.cy;
      if (std::abs(dx * c + dy * s) <= hl && std::abs(-dx * s + dy * c) <= hw) cells_[index(i, j)] = 1;
    }
  }
  clearance_.clear();
}

void OccupancyGrid::computeClearance() {
  std::vector<double> f(n_), d;
  std::vector<double> sq(static_cast<size_t>(n_) * n_);
  for (int j = 0; j < n_; ++j) {
    for (int i = 0; i < n_; ++i) f[i] = cells_[index(i, j)] ? 0.0 : kInf;
    distanceTransform1d(f, d);
    for (int i = 0; i < n_; ++i) sq[index(i, j)] = d[i];
  }
  clearance_.assign(sq.size(), 0.0f);
  for (int i = 0; i < n_; ++i) {
    for (int j = 0; j < n_; ++j) f[j] = sq[index(i, j)];
    distanceTransform1d(f, d);
    // From the nearest occupied cell's center to its edge.
    for (int j = 0; j < n_; ++j)
      clearance_[index(i, j)] = static_cast<float>(std::max(0.0, (std::sqrt(d[j]) - 0.5) * res_));
  }
}

double OccupancyGrid::clearance(double x, double y) const {
  if (!inside(x, y) || clearance_.empty()) return 0.0;
  int i, j;
  cell(x, y, i, j);
  const double border = std::min({x - x0_, y - y0_, x0_ + n_ * res_ - x, y0_ + n_ * res_ - y});
  return std::min(static_cast<double>(clearance_[index(i, j)]), border);
}

bool freeSpaceCollisionFree(const OccupancyGrid& grid, const FreeSpacePose& pose,
                            const VehicleParams& vehicle, double margin) {
  return footprintFree(grid, pose, footprint(vehicle), margin);
}

// ---------------------------------------------------------------- Dubins

std::vector<FreeSpacePose> dubinsPath(const FreeSpacePose& from, const FreeSpacePose& to, double radius,
                                      double ds) {
  const double dx = to.x - from.x, dy = to.y - from.y;
  const double d = std::hypot(dx, dy) / radius;
  const double theta = mod2pi(std::atan2(dy, dx));
  const double a = mod2pi(from.yaw - theta), b = mod2pi(to.yaw - theta);
  const double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b), cab = std::cos(a - b);

  // Each word: segment curvature signs (+1 left, 0 straight, -1 right) and
  // normalized lengths; NaN lengths mark words without a solution.
  struct Word {
    std::array<int, 3> dirs;
    std::array<double, 3> len;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<Word> words;
  {  // LSL
    const double p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
    const double t1 = std::atan2(cb - ca, d + sa - sb);
    words.push_back({{1, 0, 1}, {mod2pi(-a + t1), p2 >= 0 ? std::sqrt(p2) : nan, mod2pi(b - t1)}});
  }
  {  // RSR
    const double p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
    const double t1 = std::atan2(ca - cb, d - sa + sb);
    words.push_back({{-1, 0, -1}, {mod2pi(a - t1), p2 >= 0 ? std::sqrt(p2) : nan, mod2pi(-b + t1)}});
  }
  {  // LSR
    const double p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
    if (p2 >= 0) {
      const double p = std::sqrt(p2);
      const double t2 = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
      words.push_back({{1, 0, -1}, {mod2pi(-a + t2), p, mod2pi(-b + t2)}});
    }
  }
  {  // RSL
    const double p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb);
    if (p2 >= 0) {
      const double p = std::sqrt(p2);
      const double t2 = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
      words.push_back({{-1, 0, 1}, {mod2pi(a - t2), p, mod2pi(b - t2)}});
    }
  }
  {  // RLR
    const double c = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
    if (std::abs(c) <= 1) {
      const double p = mod2pi(kTwoPi - std::acos(c));
      const double t = mod2pi(a - std::atan2(ca - cb, d - sa + sb) + p / 2);
      words.push_back({{-1, 1, -1}, {t, p, mod2pi(a - b - t + p)}});
    }
  }
  {  // LRL
    const double c = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8;
    if (std::abs(c) <= 1) {
      const double p = mod2pi(kTwoPi - std::acos(c));
      const double t = mod2pi(-a - std::atan2(ca - cb, d + sa - sb) + p / 2);
      words.push_back({{1, -1, 1}, {t, p, mod2pi(b - a - t + p)}});
    }
  }

  // Shortest word whose end actually meets the goal pose.
  const FreeSpacePose start{from.x, from.y, from.yaw, false};
  double best = kInf;
  const Word* chosen = nullptr;
  for (const Word& w : words) {
    if (std::isnan(w.len[1])) continue;
    const double total = w.len[0] + w.len[1] + w.len[2];
    if (total >= best) continue;
    FreeSpacePose p = start;
    for (int k = 0; k < 3; ++k) p = drive(p, w.dirs[k] / radius, w.len[k] * radius);
    if (std::hypot(p.x - to.x, p.y - to.y) > 1e-3 * std::max(1.0, radius) ||
        std::abs(wrapAngle(p.yaw - to.yaw)) > 1e-3)
      continue;
    best = total;
    chosen = &w;
  }
  std::vector<FreeSpacePose> path;
  if (!chosen) return path;
  path.push_back(start);
  FreeSpacePose p = start;
  for (int k = 0; k < 3; ++k) {
    const double len = chosen->len[k] * radius;
    const int n = static_cast<int>(std::ceil(len / ds));
    const FreeSpacePose seg0 = p;
    for (int i = 1; i <= n; ++i) {
      p = drive(seg0, chosen->dirs[k] / radius, len * i / n);
      path.push_back(p);
    }
  }
  return path;
}

// ---------------------------------------------------------------- Hybrid A*

namespace {

struct Node {
  FreeSpacePose rear;
  double g = 0.0;
  int parent = -1;
  double steer = 0.0;  // curvature of the primitive that reached this node
  double ds = 0.0;     // its signed length
};

// Driving distance from every cell to the goal around obstacles, ignoring
// the heading (holonomic heuristic); infinite where unreachable.
std::vector<double> holonomicDistance(const OccupancyGrid& grid, const FreeSpacePose& goal,
                                      double clearance) {
  const int n = grid.size();
  std::vector<double> dist(static_cast<size_t>(n) * n, kInf);
  int gi, gj;
  grid.cell(goal.x, goal.y, gi, gj);
  if (gi < 0 || gj < 0 || gi >= n || gj >= n) return dist;
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
  dist[gj * n + gi] = 0.0;
  open.push({0.0, gj * n + gi});
  const double res = grid.resolution();
  constexpr std::array<std::array<int, 2>, 8> kNeighbours{
      {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  while (!open.empty()) {
    const auto [g, id] = open.top();
    open.pop();
    if (g > dist[id]) continue;
    const int i = id % n, j = id / n;
    for (const auto& nb : kNeighbours) {
      const int ni = i + nb[0], nj = j + nb[1];
      if (ni < 0 || nj < 0 || ni >= n || nj >= n) continue;
      const Vec2 c = grid.center(ni, nj);
      if (grid.clearance(c.x, c.y) < clearance) continue;
      const double ng = g + res * ((nb[0] != 0 && nb[1] != 0) ? M_SQRT2 : 1.0);
      if (ng < dist[nj * n + ni]) {
        dist[nj * n + ni] = ng;
        open.push({ng, nj * n + ni});
      }
    }
  }
  return dist;
}

}  // namespace

std::optional<std::vector<FreeSpacePose>> planHybridAStar(const OccupancyGrid& grid,
                                                          const FreeSpacePose& start,
                                                          const FreeSpacePose& goal,
                                                          const VehicleParams& vehicle,
                                                          const FreeSpaceConfig& config) {
  const Footprint fp = footprint(vehicle);
  if (!footprintFree(grid, goal, fp, config.margin)) return std::nullopt;
  // A vehicle starting closer to something than the margin (parked tightly)
  // may keep that closer distance.
  const double margin = std::min(config.margin, std::max(0.0, footprintClearance(grid, start, fp) - 0.01));

  const double k_max = vehicle.maxCurvature();
  const double radius = 1.0 / k_max;
  const FreeSpacePose start_rear = toRear(start, vehicle), goal_rear = toRear(goal, vehicle);
  const int sub = std::max(1, static_cast<int>(std::ceil(config.step / kPathSpacing)));

  // Heuristic: the larger of the straight-line distance and the distance
  // around obstacles (for the vehicle center, which needs half its width).
  const std::vector<double> around = holonomicDistance(grid, goal, 0.5 * vehicle.width);
  auto heuristic = [&](const FreeSpacePose& rear) {
    const FreeSpacePose c = toCenter(rear, vehicle);
    const double euclid = std::hypot(c.x - goal.x, c.y - goal.y);
    int i, j;
    grid.cell(c.x, c.y, i, j);
    const int n = grid.size();
    double h = euclid;
    if (i >= 0 && j >= 0 && i < n && j < n && std::isfinite(around[j * n + i]))
      h = std::max(h, around[j * n + i]);
    return h;
  };

  // Search state key: position cells of half a step and heading bins.
  const double key_res = 0.5 * config.step;
  const double extent = grid.size() * grid.resolution();
  const int nk = static_cast<int>(std::ceil(extent / key_res)) + 1;
  const Vec2 origin = grid.center(0, 0);
  auto key = [&](const FreeSpacePose& rear) -> long {
    const FreeSpacePose c = toCenter(rear, vehicle);
    const int i = static_cast<int>(std::floor((c.x - origin.x) / key_res)) + 1;
    const int j = static_cast<int>(std::floor((c.y - origin.y) / key_res)) + 1;
    if (i < 0 || j < 0 || i >= nk || j >= nk) return -1;
    const int y = static_cast<int>(std::floor(mod2pi(c.yaw) / kTwoPi * config.yaw_bins)) % config.yaw_bins;
    return (static_cast<long>(j) * nk + i) * config.yaw_bins + y;
  };
  std::vector<float> best_g(static_cast<size_t>(nk) * nk * config.yaw_bins,
                            std::numeric_limits<float>::max());
  std::vector<char> closed(best_g.size(), 0);

  std::vector<Node> nodes;
  nodes.push_back({start_rear, 0.0, -1, 0.0, 0.0});
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
  open.push({heuristic(start_rear), 0});

  std::vector<double> curvatures;
  const int ns = std::max(1, config.steer_samples);
  for (int k = 0; k < ns; ++k) curvatures.push_back(ns == 1 ? 0.0 : k_max * (1.0 - 2.0 * k / (ns - 1)));
  std::vector<double> directions{1.0};
  if (config.reverse) directions.push_back(-1.0);

  auto reconstruct = [&](int last, const std::vector<FreeSpacePose>& tail) {
    std::vector<int> chain;
    for (int id = last; id >= 0; id = nodes[id].parent) chain.push_back(id);
    std::reverse(chain.begin(), chain.end());
    std::vector<FreeSpacePose> path;
    path.push_back(toCenter(nodes[chain.front()].rear, vehicle));
    for (size_t c = 1; c < chain.size(); ++c) {
      const Node& nd = nodes[chain[c]];
      const FreeSpacePose from = nodes[nd.parent].rear;
      for (int s = 1; s <= sub; ++s)
        path.push_back(toCenter(drive(from, nd.steer, nd.ds * s / sub), vehicle));
    }
    // The first pose drives in the direction of the motion that follows.
    if (path.size() > 1) path.front().reverse = path[1].reverse;
    for (size_t k = 1; k < tail.size(); ++k) path.push_back(toCenter(tail[k], vehicle));
    return path;
  };

  for (int expansions = 0; !open.empty() && expansions < config.max_expansions; ++expansions) {
    const int id = open.top().second;
    open.pop();
    const long k0 = key(nodes[id].rear);
    if (k0 >= 0) {
      if (closed[k0] && id != 0) continue;
      closed[k0] = 1;
    }
    const Node cur = nodes[id];
    const FreeSpacePose cur_center = toCenter(cur.rear, vehicle);
    const double to_goal = std::hypot(cur_center.x - goal.x, cur_center.y - goal.y);

    if (to_goal < kGoalDistance && std::abs(wrapAngle(cur_center.yaw - goal.yaw)) < kGoalYaw)
      return reconstruct(id, {});
    if (to_goal < kShotRange) {
      const std::vector<FreeSpacePose> shot = dubinsPath(cur.rear, goal_rear, radius, kPathSpacing);
      bool free = !shot.empty();
      for (size_t k = 1; free && k < shot.size(); ++k)
        free = footprintFree(grid, toCenter(shot[k], vehicle), fp, margin);
      if (free) return reconstruct(id, shot);
    }

    for (double dir : directions) {
      for (double k : curvatures) {
        // A primitive that runs into something ends at its last free pose,
        // so the search can creep within tight spaces.
        int steps = 0;
        FreeSpacePose end = cur.rear;
        for (int s = 1; s <= sub; ++s) {
          const FreeSpacePose p = drive(cur.rear, k, dir * config.step * s / sub);
          if (!footprintFree(grid, toCenter(p, vehicle), fp, margin)) break;
          end = p;
          steps = s;
        }
        if (steps == 0) continue;
        const double ds = dir * config.step * steps / sub;
        const long kk = key(end);
        if (kk < 0 || closed[kk]) continue;
        const bool switched = cur.parent >= 0 && (cur.ds < 0.0) != (ds < 0.0);
        const double g = cur.g + std::abs(ds) * (ds < 0.0 ? config.reverse_cost : 1.0) +
                         (switched ? config.switch_cost : 0.0) +
                         config.steer_cost * std::abs(k) / k_max * steps / sub;
        if (g >= best_g[kk]) continue;
        best_g[kk] = static_cast<float>(g);
        nodes.push_back({end, g, id, k, ds});
        open.push({g + heuristic(end), static_cast<int>(nodes.size()) - 1});
      }
    }
  }
  return std::nullopt;
}

// ---------------------------------------------------------------- trajectory

Trajectory freeSpaceTrajectory(const std::vector<FreeSpacePose>& path, double v0,
                               const FreeSpaceConfig& config, double dt) {
  Trajectory traj;
  if (path.empty()) return traj;

  // Speed profile per pose: stretches between changes of direction start and
  // end at rest, accelerating and braking at `accel`.
  const size_t n = path.size();
  std::vector<double> arc(n, 0.0), speed(n, 0.0);
  for (size_t i = 1; i < n; ++i)
    arc[i] = arc[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  // Pose i was reached driving in direction path[i].reverse; a stretch runs
  // from pose b to the last pose e reached in the direction of pose b + 1.
  for (size_t b = 0; b + 1 < n;) {
    const bool reverse = path[b + 1].reverse;
    size_t e = b + 1;
    while (e + 1 < n && path[e + 1].reverse == reverse) ++e;
    const double start_v = (b == 0 && (v0 < 0.0) == reverse) ? std::min(std::abs(v0), config.speed) : 0.0;
    for (size_t i = b; i <= e; ++i) {
      const double up = std::sqrt(start_v * start_v + 2.0 * config.accel * (arc[i] - arc[b]));
      const double down = std::sqrt(2.0 * config.accel * (arc[e] - arc[i]));
      speed[i] = std::min({config.speed, up, down});
    }
    speed[b] = start_v;
    b = e;
  }

  // Time at each pose, then resample at dt.
  std::vector<double> time(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    const double ds = arc[i] - arc[i - 1];
    time[i] = time[i - 1] + (ds > 0.0 ? ds / std::max(0.5 * (speed[i] + speed[i - 1]), 0.05) : 0.0);
  }
  const double total = time.back();
  const int steps = static_cast<int>(std::floor(total / dt)) + 1;
  size_t i = 0;
  for (int k = 0; k <= steps; ++k) {
    const double t = std::min(k * dt, total);
    while (i + 2 < n && time[i + 1] <= t) ++i;
    const size_t j = std::min(i + 1, n - 1);
    const double span = time[j] - time[i];
    const double u = span > 1e-9 ? std::clamp((t - time[i]) / span, 0.0, 1.0) : 0.0;
    const FreeSpacePose& a = path[i];
    const FreeSpacePose& b = path[j];
    const double sign = b.reverse ? -1.0 : 1.0;
    traj.t.push_back(k * dt);
    traj.x.push_back(a.x + u * (b.x - a.x));
    traj.y.push_back(a.y + u * (b.y - a.y));
    traj.yaw.push_back(wrapAngle(a.yaw + u * wrapAngle(b.yaw - a.yaw)));
    traj.v.push_back(sign * (speed[i] + u * (speed[j] - speed[i])));
    const double ds = arc[j] - arc[i];
    traj.kappa.push_back(ds > 1e-6 ? sign * wrapAngle(b.yaw - a.yaw) / ds : 0.0);
    traj.s.push_back(arc[i] + u * ds);
  }
  const size_t m = traj.t.size();
  traj.a.assign(m, 0.0);
  for (size_t k = 0; k + 1 < m; ++k) traj.a[k] = (std::abs(traj.v[k + 1]) - std::abs(traj.v[k])) / dt;
  if (m > 1) traj.a[m - 1] = traj.a[m - 2];
  traj.s_d.resize(m);
  for (size_t k = 0; k < m; ++k) traj.s_d[k] = std::abs(traj.v[k]);
  traj.s_dd = traj.a;
  traj.s_ddd.assign(m, 0.0);
  traj.d.assign(m, 0.0);
  traj.d_d.assign(m, 0.0);
  traj.d_dd.assign(m, 0.0);
  traj.d_ddd.assign(m, 0.0);
  traj.end = {0.0, 0.0, traj.t.back()};
  traj.is_cartesian = true;
  return traj;
}

}  // namespace fiss_planner
