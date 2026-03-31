// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../../src/metrics/epdms_aggregation.hpp"

#include <gtest/gtest.h>

namespace autoware::planning_data_analyzer::metrics
{

TEST(EpdmsAggregationTest, HumanFilterPromotesAgentMetricToOneWhenHumanIsZero)
{
  EpdmsMetricSnapshot agent;
  agent.history_comfort = 0.0;
  agent.history_comfort_available = true;

  EpdmsMetricSnapshot human;
  human.history_comfort = 0.0;
  human.history_comfort_available = true;

  const auto result = calculate_human_filter_metrics(agent, human);
  EXPECT_TRUE(result.history_comfort_filter_applied);
  EXPECT_DOUBLE_EQ(result.filtered_history_comfort, 1.0);
}

TEST(EpdmsAggregationTest, SyntheticEpdmsUsesFirstStageMetricGroups)
{
  EpdmsMetricSnapshot agent;
  agent.history_comfort = 0.5;
  agent.history_comfort_available = true;
  agent.ego_progress = 1.0;
  agent.ego_progress_available = true;
  agent.time_to_collision_within_bound = 0.75;
  agent.time_to_collision_within_bound_available = true;
  agent.lane_keeping = 0.5;
  agent.lane_keeping_available = true;
  agent.extended_comfort = 0.25;
  agent.extended_comfort_available = true;
  agent.no_at_fault_collision = 1.0;
  agent.no_at_fault_collision_available = true;
  agent.drivable_area_compliance = 0.5;
  agent.drivable_area_compliance_available = true;
  agent.driving_direction_compliance = 1.0;
  agent.driving_direction_compliance_available = true;
  agent.traffic_light_compliance = 1.0;
  agent.traffic_light_compliance_available = true;

  HumanFilterMetrics filter;
  filter.filtered_extended_comfort = 1.0;
  filter.filtered_history_comfort = 1.0;
  filter.filtered_time_to_collision_within_bound = 1.0;
  filter.filtered_lane_keeping = 1.0;
  filter.filtered_no_at_fault_collision = 1.0;
  filter.filtered_drivable_area_compliance = 1.0;
  filter.filtered_driving_direction_compliance = 1.0;
  filter.filtered_traffic_light_compliance = 1.0;

  const auto result = calculate_synthetic_epdms(agent, filter);
  EXPECT_TRUE(result.raw_available);
  EXPECT_DOUBLE_EQ(result.raw_multiplicative_metrics_prod, 0.5);
  EXPECT_DOUBLE_EQ(result.raw_weighted_metrics, 0.703125);
  EXPECT_DOUBLE_EQ(result.raw_epdms, 0.3515625);
  EXPECT_TRUE(result.human_filtered_available);
  EXPECT_DOUBLE_EQ(result.human_filtered_multiplicative_metrics_prod, 1.0);
  EXPECT_DOUBLE_EQ(result.human_filtered_weighted_metrics, 1.0);
  EXPECT_DOUBLE_EQ(result.human_filtered_epdms, 1.0);
}

}  // namespace autoware::planning_data_analyzer::metrics
