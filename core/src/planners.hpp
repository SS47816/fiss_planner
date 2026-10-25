// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Concrete search strategies. Internal header: use makePlanner() instead.
#pragma once

#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "fiss_planner/planning/planner.hpp"

namespace fiss_planner {

/// Werling et al. (2010): generate every candidate, validate all of them and
/// return the cheapest valid one.
class FopPlanner : public Planner {
 public:
  using Planner::Planner;

 protected:
  bool search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) override;
};

/// Generate every candidate, sort by cost and validate lazily until the
/// first valid one (the pipeline of Apollo's lattice planner).
class FopPlusPlanner : public Planner {
 public:
  using Planner::Planner;

 protected:
  bool search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) override;
};

/// Shared machinery of FISS and FISS+: a lazily generated grid with a queue
/// of generated-but-unvalidated candidates ordered by cost.
class GridSearchPlanner : public Planner {
 public:
  using Planner::Planner;

 protected:
  struct Node {
    bool generated = false;
    bool searched = false;
    Trajectory traj;
  };

  void resetGrid(const Context& ctx);
  /// Cost of a grid node, generating its trajectory on first access.
  /// `is_new` reports whether this call generated it.
  double nodeCost(Context& ctx, const std::array<int, 3>& idx, bool* is_new = nullptr);
  /// Unexplored node with the lowest estimated cost, or false if none is left.
  bool initialGuess(const Context& ctx, std::array<int, 3>& idx) const;
  /// Index of the cheapest generated, unvalidated candidate.
  std::array<int, 3> peekCandidate(const Context& ctx) const;
  /// Pop and validate the cheapest candidate. Returns true if it is valid.
  bool validateNextCandidate(Context& ctx, Trajectory& best);
  void collectCandidates(const Context& ctx, std::vector<Trajectory>* out) const;

  std::vector<Node> nodes_;
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      queue_;
};

/// Sun et al., "FISS", RA-L 2022: alternate a discrete gradient descent over
/// the end-state grid with validation of the best candidate found so far.
class FissPlanner : public GridSearchPlanner {
 public:
  using GridSearchPlanner::GridSearchPlanner;

 protected:
  bool search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) override;

 private:
  void descend(Context& ctx, std::array<int, 3> idx);
};

/// Sun et al., "FISS+", IROS 2023: best-first neighbour exploration on the
/// grid followed by a continuous, step-decaying refinement of the end state.
class FissPlusPlanner : public GridSearchPlanner {
 public:
  using GridSearchPlanner::GridSearchPlanner;

 protected:
  bool search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) override;

 private:
  void explore(Context& ctx, std::array<int, 3> idx);
  bool refine(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates);
};

}  // namespace fiss_planner
