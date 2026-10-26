// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fiss_planner::behavior {

enum class Status { kSuccess, kFailure, kRunning };

/// A behaviour-tree node ticked once per planning cycle with a blackboard of
/// type B. Sequence and Fallback are the usual composites; Condition and
/// Action wrap callables.
template <typename B>
class Node {
 public:
  explicit Node(std::string name) : name_(std::move(name)) {}
  virtual ~Node() = default;
  virtual Status tick(B& board) = 0;
  const std::string& name() const { return name_; }

 private:
  std::string name_;
};

template <typename B>
using NodePtr = std::unique_ptr<Node<B>>;

/// Ticks children in order until one does not succeed; succeeds if all do.
template <typename B>
class Sequence : public Node<B> {
 public:
  Sequence(std::string name, std::vector<NodePtr<B>> children)
      : Node<B>(std::move(name)), children_(std::move(children)) {}
  Status tick(B& board) override {
    for (auto& c : children_) {
      const Status s = c->tick(board);
      if (s != Status::kSuccess) return s;
    }
    return Status::kSuccess;
  }

 private:
  std::vector<NodePtr<B>> children_;
};

/// Ticks children in order until one does not fail; fails if all do.
template <typename B>
class Fallback : public Node<B> {
 public:
  Fallback(std::string name, std::vector<NodePtr<B>> children)
      : Node<B>(std::move(name)), children_(std::move(children)) {}
  Status tick(B& board) override {
    for (auto& c : children_) {
      const Status s = c->tick(board);
      if (s != Status::kFailure) return s;
    }
    return Status::kFailure;
  }

 private:
  std::vector<NodePtr<B>> children_;
};

/// Succeeds if the predicate holds, fails otherwise.
template <typename B>
class Condition : public Node<B> {
 public:
  Condition(std::string name, std::function<bool(const B&)> pred)
      : Node<B>(std::move(name)), pred_(std::move(pred)) {}
  Status tick(B& board) override { return pred_(board) ? Status::kSuccess : Status::kFailure; }

 private:
  std::function<bool(const B&)> pred_;
};

/// Runs a callable that may change the blackboard and reports its status.
template <typename B>
class Action : public Node<B> {
 public:
  Action(std::string name, std::function<Status(B&)> act) : Node<B>(std::move(name)), act_(std::move(act)) {}
  Status tick(B& board) override { return act_(board); }

 private:
  std::function<Status(B&)> act_;
};

template <typename B, typename... C>
NodePtr<B> sequence(std::string name, C&&... children) {
  std::vector<NodePtr<B>> v;
  (v.push_back(std::forward<C>(children)), ...);
  return std::make_unique<Sequence<B>>(std::move(name), std::move(v));
}

template <typename B, typename... C>
NodePtr<B> fallback(std::string name, C&&... children) {
  std::vector<NodePtr<B>> v;
  (v.push_back(std::forward<C>(children)), ...);
  return std::make_unique<Fallback<B>>(std::move(name), std::move(v));
}

template <typename B>
NodePtr<B> condition(std::string name, std::function<bool(const B&)> pred) {
  return std::make_unique<Condition<B>>(std::move(name), std::move(pred));
}

template <typename B>
NodePtr<B> action(std::string name, std::function<Status(B&)> act) {
  return std::make_unique<Action<B>>(std::move(name), std::move(act));
}

/// A behaviour that applies when its subtree succeeds and is skipped
/// otherwise: Fallback(subtree, Success).
template <typename B>
NodePtr<B> optional(std::string name, NodePtr<B> subtree) {
  return fallback<B>(std::move(name), std::move(subtree),
                     action<B>("skip", [](B&) { return Status::kSuccess; }));
}

}  // namespace fiss_planner::behavior
