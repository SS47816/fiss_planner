// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "fiss_planner/planning/config.hpp"
#include "fiss_planner/planning/local_planner.hpp"

using namespace fiss_planner;

TEST(Config, PlannerTypeNames) {
  EXPECT_EQ(plannerTypeFromString("FOP"), PlannerType::kFop);
  EXPECT_EQ(plannerTypeFromString("fop+"), PlannerType::kFopPlus);
  EXPECT_EQ(plannerTypeFromString("FISS-Plus"), PlannerType::kFissPlus);
  EXPECT_EQ(plannerTypeFromString(toString(PlannerType::kFiss)), PlannerType::kFiss);
  EXPECT_THROW(plannerTypeFromString("rrt"), std::invalid_argument);
}

TEST(Config, ParsesYamlAndKeepsDefaults) {
  VehicleParams vehicle;
  const auto cfg = parsePlannerConfig(R"(
planner:
  type: fiss
  num_threads: 4
sampling:
  num_width: 7
  max_t: 6.0
cost:
  w_jerk: 0.5
vehicle:
  length: 5.0
)",
                                      &vehicle);
  EXPECT_EQ(cfg.type, PlannerType::kFiss);
  EXPECT_EQ(cfg.num_threads, 4);
  EXPECT_EQ(cfg.sampling.num_width, 7);
  EXPECT_EQ(cfg.sampling.num_speed, SamplingConfig{}.num_speed);
  EXPECT_DOUBLE_EQ(cfg.sampling.max_t, 6.0);
  EXPECT_DOUBLE_EQ(cfg.cost.w_jerk, 0.5);
  EXPECT_DOUBLE_EQ(vehicle.length, 5.0);
}

TEST(Config, RejectsUnknownKeysAndInvalidValues) {
  EXPECT_THROW(parsePlannerConfig("sampling: {num_widht: 3}"), std::invalid_argument);
  EXPECT_THROW(parsePlannerConfig("smapling: {}"), std::invalid_argument);
  EXPECT_THROW(parsePlannerConfig("sampling: {min_t: 5.0, max_t: 3.0}"), std::invalid_argument);
  EXPECT_THROW(parsePlannerConfig("sampling: {num_width: abc}"), std::invalid_argument);
  EXPECT_NO_THROW(parsePlannerConfig(""));
}

TEST(Config, ShippedPresetsLoad) {
  for (const char* name : {"highway", "urban", "paper_fiss"}) {
    VehicleParams v;
    EXPECT_NO_THROW(loadPlannerConfig(std::string(FISS_PLANNER_CONFIG_DIR) + "/" + name + ".yaml", &v))
        << name;
  }
}

TEST(LocalConfig, ParsesSectionsAndTopLevelKeys) {
  VehicleParams vehicle;
  vehicle.length = 4.2;
  const auto cfg = parseLocalPlannerConfig(R"(
desired_speed: 20.0
envelope_retry_weight: 500
use_speed_limit: false
planner: {type: fop}
cost: {w_lane_change: 2.5}
route: {max_adjacent_lanes: 2, urgency_time: 12.0}
stitching: {latency: 0.05}
smoother: {enabled: true, w_lat_jerk: 4.0}
vehicle: {max_decel: 7.0}
)",
                                           &vehicle);
  EXPECT_DOUBLE_EQ(cfg.desired_speed, 20.0);
  EXPECT_DOUBLE_EQ(cfg.envelope_retry_weight, 500.0);
  EXPECT_FALSE(cfg.use_speed_limit);
  EXPECT_EQ(cfg.planner.type, PlannerType::kFop);
  EXPECT_DOUBLE_EQ(cfg.planner.cost.w_lane_change, 2.5);
  EXPECT_EQ(cfg.route.max_adjacent_lanes, 2);
  EXPECT_DOUBLE_EQ(cfg.route.urgency_time, 12.0);
  EXPECT_DOUBLE_EQ(cfg.stitching.latency, 0.05);
  EXPECT_TRUE(cfg.smoother.enabled);
  EXPECT_DOUBLE_EQ(cfg.smoother.w_lat_jerk, 4.0);
  EXPECT_DOUBLE_EQ(cfg.degraded_horizon, LocalPlannerConfig{}.degraded_horizon);
  EXPECT_DOUBLE_EQ(vehicle.max_decel, 7.0);
  EXPECT_DOUBLE_EQ(vehicle.length, 4.2);  // fields without a key keep their value
}

TEST(LocalConfig, MissingKeysKeepTheBase) {
  LocalPlannerConfig base;
  base.desired_speed = 8.0;
  base.planner.cost.w_jerk = 0.7;
  base.route.lookahead = 90.0;
  const auto cfg = parseLocalPlannerConfig("cost: {w_speed: 3.0}", nullptr, base);
  EXPECT_DOUBLE_EQ(cfg.planner.cost.w_speed, 3.0);
  EXPECT_DOUBLE_EQ(cfg.planner.cost.w_jerk, 0.7);
  EXPECT_DOUBLE_EQ(cfg.desired_speed, 8.0);
  EXPECT_DOUBLE_EQ(cfg.route.lookahead, 90.0);
}

TEST(LocalConfig, YamlRoundTripIsExact) {
  LocalPlannerConfig c;
  c.desired_speed = 1.0 / 3.0;
  c.recovery_decel = 2.75;
  c.use_curvature_speed_limit = false;
  c.planner.type = PlannerType::kFiss;
  c.planner.sampling.num_width = 9;
  c.planner.sampling.dt = 0.1;
  c.planner.cost.w_heuristic = 1e-7;
  c.planner.collision.ignore_rear_in_lane = true;
  c.planner.safety.crossing_time_gap = 0.0;
  c.planner.fiss_plus.decay = 0.3;
  c.route.smoothing_tolerance = 0.05;
  c.route.w_route_urgency = 12345.678;
  c.stitching.enabled = false;
  c.path.num_left_lanes = 3;
  c.smoother.w_lon_speed = 0.123456789012345;
  VehicleParams v;
  v.max_lat_accel = 2.9;
  v.length = 4.5;

  const std::string text = toYaml(c, &v);
  VehicleParams v2;
  const auto c2 = parseLocalPlannerConfig(text, &v2);
  EXPECT_EQ(toYaml(c2, &v2), text);
  EXPECT_EQ(c2.desired_speed, c.desired_speed);
  EXPECT_EQ(c2.planner.cost.w_heuristic, 1e-7);
  EXPECT_EQ(c2.smoother.w_lon_speed, c.smoother.w_lon_speed);
  EXPECT_EQ(c2.planner.type, PlannerType::kFiss);
  EXPECT_TRUE(c2.planner.collision.ignore_rear_in_lane);
  EXPECT_FALSE(c2.stitching.enabled);
  EXPECT_EQ(v2.max_lat_accel, 2.9);
  EXPECT_NE(text.find("desired_speed: 0.3333333333333333\n"), std::string::npos) << text;
  EXPECT_NE(text.find("  dt: 0.1\n"), std::string::npos) << text;
  EXPECT_NE(text.find("  w_route_urgency: 12345.678\n"), std::string::npos) << text;
  EXPECT_NE(text.find("  crossing_time_gap: 0.0\n"), std::string::npos) << text;
  EXPECT_EQ(toYaml(c).find("vehicle:"), std::string::npos);
}

TEST(LocalConfig, RejectsUnknownKeysAndInvalidValues) {
  EXPECT_THROW(parseLocalPlannerConfig("route: {lookahed: 100}"), std::invalid_argument);
  EXPECT_THROW(parseLocalPlannerConfig("desired_sped: 10"), std::invalid_argument);
  EXPECT_THROW(parseLocalPlannerConfig("route: 3"), std::invalid_argument);
  EXPECT_THROW(parseLocalPlannerConfig("desired_speed: fast"), std::invalid_argument);
  EXPECT_THROW(parseLocalPlannerConfig("route: {station_ds: 0.0}"), std::invalid_argument);
  EXPECT_THROW(parseLocalPlannerConfig("recovery_decel: -1.0"), std::invalid_argument);
  EXPECT_THROW(parseLocalPlannerConfig("sampling: {min_t: 5.0, max_t: 3.0}"), std::invalid_argument);
  EXPECT_NO_THROW(parseLocalPlannerConfig(""));
  // The planner-only loader keeps to the planner sections.
  EXPECT_THROW(parsePlannerConfig("route: {lookahead: 100}"), std::invalid_argument);
  EXPECT_THROW(parsePlannerConfig("desired_speed: 10"), std::invalid_argument);
}

TEST(LocalConfig, ShippedPresetsAndProfilesLoad) {
  for (const char* name : {"highway", "urban", "paper_fiss"}) {
    EXPECT_NO_THROW(loadLocalPlannerConfig(std::string(FISS_PLANNER_CONFIG_DIR) + "/" + name + ".yaml"))
        << name;
  }
  for (const char* name : {"commonroad", "carla_scenarios", "nuplan", "bench2drive"}) {
    const std::string path = std::string(FISS_PLANNER_PROFILE_DIR) + "/" + name + ".yaml";
    LocalPlannerConfig cfg;
    EXPECT_NO_THROW(cfg = loadLocalPlannerConfig(path)) << name;
    EXPECT_EQ(toYaml(parseLocalPlannerConfig(toYaml(cfg))), toYaml(cfg)) << name;
  }
}
