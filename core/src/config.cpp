// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/planning/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "fiss_planner/planning/local_planner.hpp"

namespace fiss_planner {

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
void requireLocal(bool cond, const std::string& what) {
  if (!cond) throw std::invalid_argument("LocalPlannerConfig: " + what);
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
  require(cost.w_headway >= 0 && cost.w_ttc >= 0 && cost.w_clearance >= 0,
          "proximity weights must be non-negative");
  require(cost.desired_gap > 0 && cost.desired_time_headway >= 0 && cost.desired_ttc > 0 &&
              cost.desired_clearance > 0,
          "desired gap, ttc and clearance must be positive");
  require(cost.follow_time >= 0, "cost.follow_time must be >= 0");
  require(safety.min_gap >= 0 && safety.min_time_headway >= 0 && safety.min_ttc >= 0 &&
              safety.reaction_time >= 0 && safety.crossing_time_gap >= 0,
          "safety thresholds must be >= 0");
  require(collision.margin_lon >= 0 && collision.margin_lat >= 0, "margins must be >= 0");
  require(collision.stride >= 1, "collision.stride must be >= 1");
  require(fiss_plus.max_refine_iters >= 0, "fiss_plus.max_refine_iters must be >= 0");
  require(fiss_plus.decay > 0.0 && fiss_plus.decay <= 1.0, "fiss_plus.decay must be in (0, 1]");
  require(fiss_plus.time_budget >= 0.0, "fiss_plus.time_budget must be >= 0");
  require(num_threads >= 1, "num_threads must be >= 1");
}

void LocalPlannerConfig::validate() const {
  planner.validate();
  const auto& r = route;
  requireLocal(r.lookahead > 0 && r.lookbehind >= 0 && r.station_ds > 0,
               "need route.lookahead > 0, route.lookbehind >= 0 and route.station_ds > 0");
  requireLocal(r.smoothing_length >= 0 && (r.smoothing_length == 0 || r.smoothing_tolerance > 0),
               "need route.smoothing_length >= 0 and route.smoothing_tolerance > 0");
  requireLocal(r.max_adjacent_lanes >= 0, "route.max_adjacent_lanes must be >= 0");
  requireLocal(r.w_route_urgency >= 0 && r.urgency_distance > 0 && r.urgency_time >= 0,
               "need route.w_route_urgency >= 0, route.urgency_distance > 0 and route.urgency_time >= 0");
  requireLocal(
      stitching.latency >= 0 && stitching.max_position_error >= 0 && stitching.max_heading_error >= 0,
      "stitching latency and thresholds must be >= 0");
  requireLocal(path.lane_width > 0 && path.num_left_lanes >= 0 && path.num_right_lanes >= 0,
               "need path.lane_width > 0 and lane counts >= 0");
  const auto& q = smoother;
  requireLocal(q.margin_lat >= 0 && q.margin_lon >= 0, "smoother margins must be >= 0");
  requireLocal(q.w_lat_ref >= 0 && q.w_lat_target >= 0 && q.w_lat_accel >= 0 && q.w_lat_jerk >= 0 &&
                   q.w_lon_ref >= 0 && q.w_lon_speed_ref >= 0 && q.w_lon_speed >= 0 && q.w_lon_accel >= 0 &&
                   q.w_lon_jerk >= 0,
               "smoother weights must be non-negative");
  requireLocal(desired_speed >= 0 && emergency_decel >= 0 && degraded_horizon >= 0 &&
                   envelope_retry_weight >= 0 && recovery_decel >= 0,
               "desired_speed, emergency_decel, degraded_horizon, envelope_retry_weight and "
               "recovery_decel must be >= 0");
}

namespace {

/// One YAML key: reads a value into its field and writes the field back.
struct Field {
  std::function<void(const YAML::Node&)> read;
  std::function<std::string()> write;
};
using Fields = std::vector<std::pair<std::string, Field>>;

/// A YAML map of fields; the section without a name holds top-level keys.
struct Section {
  std::string name;
  Fields fields;
};

/// Shortest text that reads back as exactly `x`: fixed-point unless the
/// magnitude is extreme, always with a decimal point so that it stays a float
/// in any YAML reader.
std::string formatDouble(double x) {
  if (std::isnan(x)) return ".nan";
  if (std::isinf(x)) return x > 0 ? ".inf" : "-.inf";
  char buf[64];
  int digits = 1;
  for (; digits < 17; ++digits) {
    std::snprintf(buf, sizeof(buf), "%.*e", digits - 1, x);
    if (std::strtod(buf, nullptr) == x) break;
  }
  std::snprintf(buf, sizeof(buf), "%.*e", digits - 1, x);
  std::string s = buf;
  const int exponent = std::atoi(s.c_str() + s.find('e') + 1);
  if (exponent >= -5 && exponent < 16) {
    std::snprintf(buf, sizeof(buf), "%.*f", std::max(0, digits - 1 - exponent), x);
    s = buf;
    if (s.find('.') == std::string::npos) s += ".0";
  } else if (s.find('.') == std::string::npos) {
    s.insert(s.find('e'), ".0");
  }
  return s;
}

template <typename T>
Field field(T& f) {
  Field r;
  r.read = [&f](const YAML::Node& n) { f = n.as<T>(); };
  if constexpr (std::is_same_v<T, bool>) {
    r.write = [&f] { return std::string(f ? "true" : "false"); };
  } else if constexpr (std::is_integral_v<T>) {
    r.write = [&f] { return std::to_string(f); };
  } else {
    r.write = [&f] { return formatDouble(f); };
  }
  return r;
}

std::vector<Section> plannerSections(PlannerConfig& c) {
  auto& s = c.sampling;
  auto& w = c.cost;
  auto& k = c.constraints;
  auto& col = c.collision;
  auto& sf = c.safety;
  auto& f = c.fiss_plus;
  Field type{[&c](const YAML::Node& n) { c.type = plannerTypeFromString(n.as<std::string>()); },
             [&c] { return toString(c.type); }};
  return {
      {"planner",
       {{"type", type},
        {"num_threads", field(c.num_threads)},
        {"keep_candidates", field(c.keep_candidates)}}},
      {"sampling",
       {{"num_width", field(s.num_width)},
        {"num_speed", field(s.num_speed)},
        {"num_t", field(s.num_t)},
        {"min_t", field(s.min_t)},
        {"max_t", field(s.max_t)},
        {"dt", field(s.dt)},
        {"min_speed", field(s.min_speed)},
        {"lane_margin", field(s.lane_margin)},
        {"follow_lead", field(s.follow_lead)}}},
      {"cost",
       {{"w_time", field(w.w_time)},
        {"w_speed", field(w.w_speed)},
        {"w_accel", field(w.w_accel)},
        {"w_jerk", field(w.w_jerk)},
        {"w_lateral", field(w.w_lateral)},
        {"w_lane_change", field(w.w_lane_change)},
        {"w_heuristic", field(w.w_heuristic)},
        {"w_headway", field(w.w_headway)},
        {"desired_gap", field(w.desired_gap)},
        {"desired_time_headway", field(w.desired_time_headway)},
        {"w_ttc", field(w.w_ttc)},
        {"desired_ttc", field(w.desired_ttc)},
        {"w_clearance", field(w.w_clearance)},
        {"desired_clearance", field(w.desired_clearance)},
        {"follow_time", field(w.follow_time)}}},
      {"constraints",
       {{"check_curvature", field(k.check_curvature)},
        {"check_lat_accel", field(k.check_lat_accel)},
        {"check_road_bounds", field(k.check_road_bounds)},
        {"curvature_check_min_speed", field(k.curvature_check_min_speed)}}},
      {"collision",
       {{"margin_lon", field(col.margin_lon)},
        {"margin_lat", field(col.margin_lat)},
        {"stride", field(col.stride)},
        {"ignore_rear_in_lane", field(col.ignore_rear_in_lane)}}},
      {"safety",
       {{"enabled", field(sf.enabled)},
        {"min_gap", field(sf.min_gap)},
        {"min_time_headway", field(sf.min_time_headway)},
        {"min_ttc", field(sf.min_ttc)},
        {"reaction_time", field(sf.reaction_time)},
        {"crossing_time_gap", field(sf.crossing_time_gap)}}},
      {"fiss_plus",
       {{"refine", field(f.refine)},
        {"max_refine_iters", field(f.max_refine_iters)},
        {"decay", field(f.decay)},
        {"time_budget", field(f.time_budget)}}},
  };
}

std::vector<Section> localSections(LocalPlannerConfig& c) {
  std::vector<Section> out = {{"",
                               {{"desired_speed", field(c.desired_speed)},
                                {"use_speed_limit", field(c.use_speed_limit)},
                                {"use_curvature_speed_limit", field(c.use_curvature_speed_limit)},
                                {"emergency_decel", field(c.emergency_decel)},
                                {"degraded_horizon", field(c.degraded_horizon)},
                                {"envelope_retry_weight", field(c.envelope_retry_weight)},
                                {"recovery_decel", field(c.recovery_decel)}}}};
  auto planner = plannerSections(c.planner);
  out.insert(out.end(), planner.begin(), planner.end());
  auto& r = c.route;
  auto& st = c.stitching;
  auto& p = c.path;
  auto& q = c.smoother;
  out.push_back({"route",
                 {{"lookahead", field(r.lookahead)},
                  {"lookbehind", field(r.lookbehind)},
                  {"station_ds", field(r.station_ds)},
                  {"smoothing_length", field(r.smoothing_length)},
                  {"smoothing_tolerance", field(r.smoothing_tolerance)},
                  {"max_adjacent_lanes", field(r.max_adjacent_lanes)},
                  {"w_route_urgency", field(r.w_route_urgency)},
                  {"urgency_distance", field(r.urgency_distance)},
                  {"urgency_time", field(r.urgency_time)}}});
  out.push_back({"stitching",
                 {{"enabled", field(st.enabled)},
                  {"latency", field(st.latency)},
                  {"max_position_error", field(st.max_position_error)},
                  {"max_heading_error", field(st.max_heading_error)}}});
  out.push_back({"path",
                 {{"lane_width", field(p.lane_width)},
                  {"num_left_lanes", field(p.num_left_lanes)},
                  {"num_right_lanes", field(p.num_right_lanes)},
                  {"allow_left", field(p.allow_left)},
                  {"allow_right", field(p.allow_right)}}});
  out.push_back({"smoother",
                 {{"enabled", field(q.enabled)},
                  {"margin_lat", field(q.margin_lat)},
                  {"margin_lon", field(q.margin_lon)},
                  {"w_lat_ref", field(q.w_lat_ref)},
                  {"w_lat_target", field(q.w_lat_target)},
                  {"w_lat_accel", field(q.w_lat_accel)},
                  {"w_lat_jerk", field(q.w_lat_jerk)},
                  {"w_lon_ref", field(q.w_lon_ref)},
                  {"w_lon_speed_ref", field(q.w_lon_speed_ref)},
                  {"w_lon_speed", field(q.w_lon_speed)},
                  {"w_lon_accel", field(q.w_lon_accel)},
                  {"w_lon_jerk", field(q.w_lon_jerk)}}});
  return out;
}

Section vehicleSection(VehicleParams& v) {
  return {"vehicle",
          {{"length", field(v.length)},
           {"width", field(v.width)},
           {"wheelbase", field(v.wheelbase)},
           {"rear_axle_to_center", field(v.rear_axle_to_center)},
           {"max_speed", field(v.max_speed)},
           {"max_accel", field(v.max_accel)},
           {"max_decel", field(v.max_decel)},
           {"max_steer", field(v.max_steer)},
           {"max_steer_rate", field(v.max_steer_rate)},
           {"max_lat_accel", field(v.max_lat_accel)}}};
}

const Field* findField(const Fields& fields, const std::string& key) {
  const auto it = std::find_if(fields.begin(), fields.end(), [&](const auto& kv) { return kv.first == key; });
  return it == fields.end() ? nullptr : &it->second;
}

void readField(const Field& f, const YAML::Node& value, const std::string& path) {
  try {
    f.read(value);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument("bad value for '" + path + "': " + e.what());
  }
}

void readSection(const YAML::Node& node, const Section& section) {
  if (!node.IsMap()) throw std::invalid_argument("config section '" + section.name + "' must be a map");
  for (const auto& kv : node) {
    const auto key = kv.first.as<std::string>();
    const auto path = section.name + "." + key;
    const Field* f = findField(section.fields, key);
    if (!f) throw std::invalid_argument("unknown config key '" + path + "'");
    readField(*f, kv.second, path);
  }
}

void read(const YAML::Node& root, const std::vector<Section>& sections) {
  if (!root || root.IsNull()) return;
  if (!root.IsMap()) throw std::invalid_argument("config root must be a map");
  for (const auto& kv : root) {
    const auto key = kv.first.as<std::string>();
    bool found = false;
    for (const auto& section : sections) {
      if (section.name.empty()) {
        if (const Field* f = findField(section.fields, key)) {
          readField(*f, kv.second, key);
          found = true;
        }
      } else if (section.name == key) {
        readSection(kv.second, section);
        found = true;
      }
      if (found) break;
    }
    if (!found) throw std::invalid_argument("unknown config section '" + key + "'");
  }
}

std::string write(const std::vector<Section>& sections) {
  std::string out;
  for (const auto& section : sections) {
    const std::string indent = section.name.empty() ? "" : "  ";
    if (!section.name.empty()) out += section.name + ":\n";
    for (const auto& [key, f] : section.fields) out += indent + key + ": " + f.write() + "\n";
  }
  return out;
}

YAML::Node loadFile(const std::string& yaml_path) {
  try {
    return YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument("cannot read config '" + yaml_path + "': " + e.what());
  }
}

YAML::Node loadText(const std::string& yaml_text) {
  try {
    return YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("cannot parse config: ") + e.what());
  }
}

PlannerConfig parse(const YAML::Node& root, VehicleParams* vehicle) {
  PlannerConfig c;
  VehicleParams scratch;
  auto sections = plannerSections(c);
  sections.push_back(vehicleSection(vehicle ? *vehicle : scratch));
  read(root, sections);
  c.validate();
  return c;
}

LocalPlannerConfig parseLocal(const YAML::Node& root, VehicleParams* vehicle,
                              const LocalPlannerConfig& base) {
  LocalPlannerConfig c = base;
  VehicleParams scratch;
  auto sections = localSections(c);
  sections.push_back(vehicleSection(vehicle ? *vehicle : scratch));
  read(root, sections);
  c.validate();
  return c;
}

}  // namespace

PlannerConfig loadPlannerConfig(const std::string& yaml_path, VehicleParams* vehicle) {
  return parse(loadFile(yaml_path), vehicle);
}

PlannerConfig parsePlannerConfig(const std::string& yaml_text, VehicleParams* vehicle) {
  return parse(loadText(yaml_text), vehicle);
}

LocalPlannerConfig loadLocalPlannerConfig(const std::string& yaml_path, VehicleParams* vehicle,
                                          const LocalPlannerConfig& base) {
  return parseLocal(loadFile(yaml_path), vehicle, base);
}

LocalPlannerConfig parseLocalPlannerConfig(const std::string& yaml_text, VehicleParams* vehicle,
                                           const LocalPlannerConfig& base) {
  return parseLocal(loadText(yaml_text), vehicle, base);
}

std::string toYaml(const LocalPlannerConfig& config, const VehicleParams* vehicle) {
  LocalPlannerConfig c = config;
  auto sections = localSections(c);
  VehicleParams v = vehicle ? *vehicle : VehicleParams{};
  if (vehicle) sections.push_back(vehicleSection(v));
  return write(sections);
}

}  // namespace fiss_planner
