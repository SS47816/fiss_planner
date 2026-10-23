// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/planning/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <stdexcept>

namespace frenet_planner {

std::string toString(PlannerType type) {
  switch (type) {
    case PlannerType::kFop:
      return "fop";
    case PlannerType::kFopPlus:
      return "fop_plus";
    case PlannerType::kFiss:
      return "fiss";
    case PlannerType::kFissPlus:
      return "fiss_plus";
  }
  return "unknown";
}

PlannerType plannerTypeFromString(const std::string& name) {
  std::string n;
  for (char c : name) {
    if (c == '-' || c == ' ') continue;
    n.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (n == "fop") return PlannerType::kFop;
  if (n == "fop+" || n == "fop_plus" || n == "fopplus") return PlannerType::kFopPlus;
  if (n == "fiss") return PlannerType::kFiss;
  if (n == "fiss+" || n == "fiss_plus" || n == "fissplus") return PlannerType::kFissPlus;
  throw std::invalid_argument("unknown planner type '" + name +
                              "' (expected fop, fop_plus, fiss, fiss_plus)");
}

namespace {
void require(bool cond, const std::string& what) {
  if (!cond) throw std::invalid_argument("PlannerConfig: " + what);
}
}  // namespace

void PlannerConfig::validate() const {
  const auto& s = sampling;
  require(s.num_width >= 1 && s.num_speed >= 1 && s.num_t >= 1, "sample counts must be >= 1");
  require(s.min_t > 0.0 && s.max_t >= s.min_t, "need 0 < min_t <= max_t");
  require(s.dt > 0.0 && s.dt < s.min_t, "need 0 < dt < min_t");
  require(s.min_speed >= 0.0, "min_speed must be >= 0");
  require(cost.w_time >= 0 && cost.w_speed >= 0 && cost.w_accel >= 0 && cost.w_jerk >= 0 &&
              cost.w_lateral >= 0 && cost.w_lane_change >= 0 && cost.w_heuristic >= 0,
          "cost weights must be non-negative");
  require(collision.margin_lon >= 0 && collision.margin_lat >= 0, "margins must be >= 0");
  require(collision.stride >= 1, "collision.stride must be >= 1");
  require(fiss_plus.max_refine_iters >= 0, "fiss_plus.max_refine_iters must be >= 0");
  require(fiss_plus.decay > 0.0 && fiss_plus.decay <= 1.0, "fiss_plus.decay must be in (0, 1]");
  require(fiss_plus.time_budget >= 0.0, "fiss_plus.time_budget must be >= 0");
  require(num_threads >= 1, "num_threads must be >= 1");
}

namespace {

using Setter = std::function<void(const YAML::Node&)>;

template <typename T>
Setter set(T& field) {
  return [&field](const YAML::Node& n) { field = n.as<T>(); };
}

void apply(const YAML::Node& node, const std::string& section, const std::map<std::string, Setter>& fields) {
  if (!node) return;
  if (!node.IsMap()) throw std::invalid_argument("config section '" + section + "' must be a map");
  for (const auto& kv : node) {
    const auto key = kv.first.as<std::string>();
    const auto it = fields.find(key);
    if (it == fields.end()) {
      throw std::invalid_argument("unknown config key '" + section + "." + key + "'");
    }
    try {
      it->second(kv.second);
    } catch (const YAML::Exception& e) {
      throw std::invalid_argument("bad value for '" + section + "." + key + "': " + e.what());
    }
  }
}

PlannerConfig parse(const YAML::Node& root, VehicleParams* vehicle) {
  PlannerConfig c;
  if (!root || root.IsNull()) return c;
  if (!root.IsMap()) throw std::invalid_argument("config root must be a map");
  static const char* kSections[] = {"planner",   "sampling",  "cost",   "constraints",
                                    "collision", "fiss_plus", "vehicle"};
  for (const auto& kv : root) {
    const auto key = kv.first.as<std::string>();
    if (std::find(std::begin(kSections), std::end(kSections), key) == std::end(kSections)) {
      throw std::invalid_argument("unknown config section '" + key + "'");
    }
  }

  apply(root["planner"], "planner",
        {{"type", [&c](const YAML::Node& n) { c.type = plannerTypeFromString(n.as<std::string>()); }},
         {"num_threads", set(c.num_threads)},
         {"keep_candidates", set(c.keep_candidates)}});
  auto& s = c.sampling;
  apply(root["sampling"], "sampling",
        {{"num_width", set(s.num_width)},
         {"num_speed", set(s.num_speed)},
         {"num_t", set(s.num_t)},
         {"min_t", set(s.min_t)},
         {"max_t", set(s.max_t)},
         {"dt", set(s.dt)},
         {"min_speed", set(s.min_speed)},
         {"lane_margin", set(s.lane_margin)}});
  auto& w = c.cost;
  apply(root["cost"], "cost",
        {{"w_time", set(w.w_time)},
         {"w_speed", set(w.w_speed)},
         {"w_accel", set(w.w_accel)},
         {"w_jerk", set(w.w_jerk)},
         {"w_lateral", set(w.w_lateral)},
         {"w_lane_change", set(w.w_lane_change)},
         {"w_heuristic", set(w.w_heuristic)}});
  auto& k = c.constraints;
  apply(root["constraints"], "constraints",
        {{"check_curvature", set(k.check_curvature)},
         {"check_lat_accel", set(k.check_lat_accel)},
         {"check_road_bounds", set(k.check_road_bounds)},
         {"curvature_check_min_speed", set(k.curvature_check_min_speed)}});
  auto& col = c.collision;
  apply(root["collision"], "collision",
        {{"margin_lon", set(col.margin_lon)},
         {"margin_lat", set(col.margin_lat)},
         {"stride", set(col.stride)}});
  auto& f = c.fiss_plus;
  apply(root["fiss_plus"], "fiss_plus",
        {{"refine", set(f.refine)},
         {"max_refine_iters", set(f.max_refine_iters)},
         {"decay", set(f.decay)},
         {"time_budget", set(f.time_budget)}});

  VehicleParams scratch;
  auto& v = vehicle ? *vehicle : scratch;
  apply(root["vehicle"], "vehicle",
        {{"length", set(v.length)},
         {"width", set(v.width)},
         {"wheelbase", set(v.wheelbase)},
         {"rear_axle_to_center", set(v.rear_axle_to_center)},
         {"max_speed", set(v.max_speed)},
         {"max_accel", set(v.max_accel)},
         {"max_decel", set(v.max_decel)},
         {"max_steer", set(v.max_steer)},
         {"max_steer_rate", set(v.max_steer_rate)},
         {"max_lat_accel", set(v.max_lat_accel)}});
  c.validate();
  return c;
}

}  // namespace

PlannerConfig loadPlannerConfig(const std::string& yaml_path, VehicleParams* vehicle) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument("cannot read config '" + yaml_path + "': " + e.what());
  }
  return parse(root, vehicle);
}

PlannerConfig parsePlannerConfig(const std::string& yaml_text, VehicleParams* vehicle) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("cannot parse config: ") + e.what());
  }
  return parse(root, vehicle);
}

}  // namespace frenet_planner
