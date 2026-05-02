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

#include "../../src/metrics/epdms/subscores/drivable_area_compliance.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <vector>

namespace
{

autoware_planning_msgs::msg::Trajectory make_trajectory(const double lateral_offset)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "map";

  for (std::size_t i = 0; i < 3; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = 3.0 + static_cast<double>(i) * 2.0;
    point.pose.position.y = lateral_offset;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0;
    point.time_from_start.nanosec = static_cast<uint32_t>(i * 100000000UL);
    trajectory.points.push_back(point);
  }

  return trajectory;
}

lanelet::ConstLanelets make_drivable_lanelets()
{
  lanelet::LineString3d left_bound{
    lanelet::InvalId,
    {lanelet::Point3d{lanelet::InvalId, -5.0, 2.0, 0.0},
     lanelet::Point3d{lanelet::InvalId, 12.0, 2.0, 0.0}}};
  lanelet::LineString3d right_bound{
    lanelet::InvalId,
    {lanelet::Point3d{lanelet::InvalId, -5.0, -2.0, 0.0},
     lanelet::Point3d{lanelet::InvalId, 12.0, -2.0, 0.0}}};
  return lanelet::ConstLanelets{lanelet::ConstLanelet{lanelet::InvalId, left_bound, right_bound}};
}

autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 2.5, 1.6, 1.0, 1.0, 0.5, 0.5, 1.8, 0.7);
}

}  // namespace

TEST(DrivableAreaComplianceTest, ReturnsOneWhenFullFootprintStaysInsideFusedDrivableArea)
{
  const auto result = autoware::planning_data_analyzer::metrics::calculate_drivable_area_compliance(
    make_trajectory(0.0), make_drivable_lanelets(), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "compliant");
}

TEST(DrivableAreaComplianceTest, ReturnsZeroWhenAnyFootprintLeavesFusedDrivableArea)
{
  const auto result = autoware::planning_data_analyzer::metrics::calculate_drivable_area_compliance(
    make_trajectory(1.5), make_drivable_lanelets(), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "non_compliant_footprint_outside_drivable_area");
}

TEST(DrivableAreaComplianceTest, ReturnsUnavailableWhenRequiredInputsAreMissing)
{
  const auto result = autoware::planning_data_analyzer::metrics::calculate_drivable_area_compliance(
    make_trajectory(0.0), {}, autoware::vehicle_info_utils::VehicleInfo{});

  EXPECT_FALSE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "unavailable_no_drivable_lanelets");
}
