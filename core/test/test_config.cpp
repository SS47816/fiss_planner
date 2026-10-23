// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "frenet_planner/planning/config.hpp"

using namespace frenet_planner;

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
    EXPECT_NO_THROW(loadPlannerConfig(std::string(FRENET_PLANNER_CONFIG_DIR) + "/" + name + ".yaml", &v))
        << name;
  }
}
