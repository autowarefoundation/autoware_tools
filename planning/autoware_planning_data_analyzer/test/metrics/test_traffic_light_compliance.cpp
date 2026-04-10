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

#include "../../src/metrics/traffic_light_compliance.hpp"

#include <gtest/gtest.h>

TEST(TrafficLightComplianceTest, EmptyTrajectoryIsUnavailable)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  const auto result = autoware::planning_data_analyzer::metrics::calculate_traffic_light_compliance(
    trajectory, nullptr, nullptr, autoware::vehicle_info_utils::VehicleInfo{});

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_empty_trajectory");
}
