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

#include "metrics/epdms/subscores/traffic_light_compliance.hpp"

#include <gtest/gtest.h>

TEST(TrafficLightComplianceTest, EmptyTrajectoryIsUnavailable)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  const auto result = autoware::planning_data_analyzer::metrics::calculate_traffic_light_compliance(
    trajectory, nullptr, nullptr, autoware::vehicle_info_utils::VehicleInfo{});

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_empty_trajectory");
}

TEST(TrafficLightComplianceTest, MissingRouteHandlerIsUnavailable)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.points.resize(1);
  trajectory.points.front().pose.orientation.w = 1.0;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  vehicle_info.vehicle_length_m = 4.8;
  vehicle_info.vehicle_width_m = 1.8;
  vehicle_info.wheel_base_m = 2.8;
  vehicle_info.front_overhang_m = 1.0;
  vehicle_info.rear_overhang_m = 1.0;
  vehicle_info.left_overhang_m = 0.9;
  vehicle_info.right_overhang_m = 0.9;

  const auto result = autoware::planning_data_analyzer::metrics::calculate_traffic_light_compliance(
    trajectory, nullptr, nullptr, vehicle_info);

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_no_route_handler");
}
