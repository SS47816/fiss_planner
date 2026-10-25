// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "planners.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace fiss_planner {

namespace {
void convertAll(const ReferenceLine& ref, std::vector<Trajectory>& trajs) {
  for (auto& t : trajs) {
    if (!t.is_cartesian) convertToCartesian(ref, t);
  }
}
}  // namespace

// ----------------------------------------------------------------------------
// FOP
// ----------------------------------------------------------------------------

bool FopPlanner::search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) {
  const auto n = ctx.grid.dims();
  std::vector<Trajectory> trajs;
  trajs.reserve(static_cast<size_t>(ctx.grid.numSamples()));
  for (int i = 0; i < n[0]; ++i)
    for (int j = 0; j < n[1]; ++j)
      for (int k = 0; k < n[2]; ++k) trajs.push_back(generate(ctx, i, j, k));

  const auto ok = validateAll(ctx, trajs, ctx.stats);
  int best_i = -1;
  for (size_t i = 0; i < trajs.size(); ++i) {
    if (ok[i] && (best_i < 0 || trajs[i].totalCost() < trajs[best_i].totalCost())) {
      best_i = static_cast<int>(i);
    }
  }
  if (best_i >= 0) best = trajs[static_cast<size_t>(best_i)];
  if (candidates) *candidates = std::move(trajs);
  return best_i >= 0;
}

// ----------------------------------------------------------------------------
// FOP+
// ----------------------------------------------------------------------------

bool FopPlusPlanner::search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) {
  const auto n = ctx.grid.dims();
  std::vector<Trajectory> trajs;
  trajs.reserve(static_cast<size_t>(ctx.grid.numSamples()));
  for (int i = 0; i < n[0]; ++i)
    for (int j = 0; j < n[1]; ++j)
      for (int k = 0; k < n[2]; ++k) trajs.push_back(generate(ctx, i, j, k));

  std::vector<size_t> order(trajs.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(),
                   [&](size_t a, size_t b) { return trajs[a].totalCost() < trajs[b].totalCost(); });

  // Validate in cost order, one batch of num_threads candidates at a time.
  const size_t batch = static_cast<size_t>(std::max(1, config_.num_threads));
  bool found = false;
  for (size_t start = 0; start < order.size() && !found; start += batch) {
    const size_t end = std::min(order.size(), start + batch);
    std::vector<Trajectory> group;
    for (size_t r = start; r < end; ++r) group.push_back(std::move(trajs[order[r]]));
    const auto ok = validateAll(ctx, group, ctx.stats);
    for (size_t r = start; r < end; ++r) trajs[order[r]] = std::move(group[r - start]);
    for (size_t r = start; r < end; ++r) {
      if (ok[r - start]) {
        best = trajs[order[r]];
        found = true;
        break;
      }
    }
  }
  if (candidates) {
    convertAll(*ctx.input.reference, trajs);
    *candidates = std::move(trajs);
  }
  return found;
}

// ----------------------------------------------------------------------------
// Lazy grid shared by FISS and FISS+
// ----------------------------------------------------------------------------

void GridSearchPlanner::resetGrid(const Context& ctx) {
  nodes_.assign(static_cast<size_t>(ctx.grid.numSamples()), Node{});
  queue_ = {};
}

double GridSearchPlanner::nodeCost(Context& ctx, const std::array<int, 3>& idx, bool* is_new) {
  Node& node = nodes_[static_cast<size_t>(ctx.grid.flat(idx[0], idx[1], idx[2]))];
  if (is_new) *is_new = !node.generated;
  if (!node.generated) {
    node.traj = generate(ctx, idx[0], idx[1], idx[2]);
    node.generated = true;
    queue_.push({node.traj.totalCost(), ctx.grid.flat(idx[0], idx[1], idx[2])});
  }
  return node.traj.totalCost();
}

bool GridSearchPlanner::initialGuess(const Context& ctx, std::array<int, 3>& idx) const {
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  const auto n = ctx.grid.dims();
  for (int i = 0; i < n[0]; ++i)
    for (int j = 0; j < n[1]; ++j)
      for (int k = 0; k < n[2]; ++k) {
        if (nodes_[static_cast<size_t>(ctx.grid.flat(i, j, k))].generated) continue;
        const double c = estimate(ctx, i, j, k);
        if (c < best) {
          best = c;
          idx = {i, j, k};
          found = true;
        }
      }
  return found;
}

std::array<int, 3> GridSearchPlanner::peekCandidate(const Context& ctx) const {
  return ctx.grid.unflat(queue_.top().second);
}

bool GridSearchPlanner::validateNextCandidate(Context& ctx, Trajectory& best) {
  const int f = queue_.top().second;
  queue_.pop();
  Trajectory& traj = nodes_[static_cast<size_t>(f)].traj;
  if (validate(ctx, traj, ctx.stats)) {
    best = traj;
    return true;
  }
  return false;
}

void GridSearchPlanner::collectCandidates(const Context& ctx, std::vector<Trajectory>* out) const {
  if (!out) return;
  for (const auto& node : nodes_) {
    if (node.generated) out->push_back(node.traj);
  }
  convertAll(*ctx.input.reference, *out);
}

// ----------------------------------------------------------------------------
// FISS
// ----------------------------------------------------------------------------

// Descend from `idx` until reaching a node that has already been searched.
// At each node the cost gradient is estimated along every dimension from one
// neighbour, and the search steps along the steepest dimension.
void FissPlanner::descend(Context& ctx, std::array<int, 3> idx) {
  const auto n = ctx.grid.dims();
  while (true) {
    Node& node = nodes_[static_cast<size_t>(ctx.grid.flat(idx[0], idx[1], idx[2]))];
    if (node.searched) return;
    node.searched = true;

    const double center = nodeCost(ctx, idx);
    double grad[3] = {0.0, 0.0, 0.0};
    for (int dim = 0; dim < 3; ++dim) {
      if (n[dim] <= 1) continue;
      auto nb = idx;
      if (idx[dim] < n[dim] - 1) {  // forward difference
        ++nb[dim];
        grad[dim] = nodeCost(ctx, nb) - center;
        if (grad[dim] >= 0.0 && idx[dim] == 0) grad[dim] = 0.0;  // no lower neighbour to go to
      } else {                                                   // backward difference at the upper boundary
        --nb[dim];
        grad[dim] = center - nodeCost(ctx, nb);
        if (grad[dim] <= 0.0) grad[dim] = 0.0;  // cannot move further up
      }
    }
    int dim = 0;
    for (int d = 1; d < 3; ++d) {
      if (std::abs(grad[d]) > std::abs(grad[dim])) dim = d;
    }
    if (grad[dim] == 0.0) return;  // local minimum
    idx[dim] += grad[dim] > 0.0 ? -1 : 1;
  }
}

bool FissPlanner::search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) {
  resetGrid(ctx);
  bool found = false;
  while (!found) {
    ++ctx.stats.num_iterations;
    std::array<int, 3> idx;
    if (queue_.empty()) {
      if (!initialGuess(ctx, idx)) break;  // every sample explored and rejected
    } else {
      idx = peekCandidate(ctx);
    }
    descend(ctx, idx);
    if (queue_.empty()) break;
    found = validateNextCandidate(ctx, best);
  }
  collectCandidates(ctx, candidates);
  return found;
}

// ----------------------------------------------------------------------------
// FISS+
// ----------------------------------------------------------------------------

// Best-first exploration: expand all six neighbours of a node and keep
// expanding newly generated neighbours that are no worse than their parent.
void FissPlusPlanner::explore(Context& ctx, std::array<int, 3> idx) {
  const auto n = ctx.grid.dims();
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> frontier;
  while (true) {
    const double center = nodeCost(ctx, idx);
    for (int dim = 0; dim < 3; ++dim) {
      for (int step : {-1, 1}) {
        auto nb = idx;
        nb[dim] += step;
        if (nb[dim] < 0 || nb[dim] >= n[dim]) continue;
        bool is_new = false;
        const double c = nodeCost(ctx, nb, &is_new);
        if (is_new && c <= center) frontier.push({c, ctx.grid.flat(nb[0], nb[1], nb[2])});
      }
    }
    if (frontier.empty()) return;
    idx = ctx.grid.unflat(frontier.top().second);
    frontier.pop();
  }
}

bool FissPlusPlanner::search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) {
  resetGrid(ctx);
  bool found = false;
  while (!found) {
    ++ctx.stats.num_iterations;
    std::array<int, 3> idx;
    if (queue_.empty()) {
      if (!initialGuess(ctx, idx)) break;
    } else {
      idx = peekCandidate(ctx);
    }
    explore(ctx, idx);
    if (queue_.empty()) break;
    found = validateNextCandidate(ctx, best);
  }
  collectCandidates(ctx, candidates);
  if (found && config_.fiss_plus.refine) refine(ctx, best, candidates);
  return found;
}

// Continuous refinement around the coarse solution: central-difference
// gradient with a step that decays every iteration, then validate refined
// candidates that improve on the coarse cost in cost order.
bool FissPlusPlanner::refine(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) {
  const auto& cfg = config_.fiss_plus;
  const auto lo = ctx.grid.lower(), hi = ctx.grid.upper();
  auto res = ctx.grid.resolution();
  const auto clampX = [&](std::array<double, 3> x) {
    for (int d = 0; d < 3; ++d) x[d] = std::clamp(x[d], lo[d], hi[d]);
    return x;
  };
  const auto toEnd = [](const std::array<double, 3>& x) { return EndState{x[0], x[1], x[2]}; };

  std::vector<Trajectory> refined;
  std::array<double, 3> x = {best.end.d, best.end.s_d, best.end.T};
  for (int it = 0; it < cfg.max_refine_iters; ++it) {
    if (cfg.time_budget > 0.0 && elapsedSeconds(ctx) >= cfg.time_budget) break;
    std::array<double, 3> grad{0.0, 0.0, 0.0};
    for (int d = 0; d < 3; ++d) {
      if (!(res[d] > 0.0)) continue;
      auto xl = x, xr = x;
      xl[d] -= res[d];
      xr[d] += res[d];
      xl = clampX(xl);
      xr = clampX(xr);
      if (!(xr[d] - xl[d] > 1e-9)) continue;
      refined.push_back(generate(ctx, toEnd(xl)));
      const double jl = refined.back().totalCost();
      refined.push_back(generate(ctx, toEnd(xr)));
      const double jr = refined.back().totalCost();
      grad[d] = (jr - jl) / (xr[d] - xl[d]);
    }
    const double norm = std::sqrt(grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2]);
    if (!(norm > 1e-12)) break;
    for (int d = 0; d < 3; ++d) {
      res[d] *= cfg.decay;
      x[d] -= res[d] * grad[d] / norm;
    }
    x = clampX(x);
    refined.push_back(generate(ctx, toEnd(x)));
  }

  std::stable_sort(refined.begin(), refined.end(),
                   [](const Trajectory& a, const Trajectory& b) { return a.totalCost() < b.totalCost(); });
  bool improved = false;
  for (auto& traj : refined) {
    if (traj.totalCost() >= best.totalCost()) break;
    if (validate(ctx, traj, ctx.stats)) {
      best = traj;
      improved = true;
      break;
    }
  }
  if (candidates) {
    convertAll(*ctx.input.reference, refined);
    candidates->insert(candidates->end(), refined.begin(), refined.end());
  }
  return improved;
}

}  // namespace fiss_planner
