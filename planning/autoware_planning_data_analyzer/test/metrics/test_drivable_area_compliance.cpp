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

#include "metrics/epdms/subscores/drivable_area_compliance.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{
namespace
{

using autoware::route_handler::RouteHandler;

geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

autoware_planning_msgs::msg::Trajectory make_trajectory(const double lateral_offset)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "map";

  for (std::size_t i = 0; i < 3; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = make_pose(3.0 + static_cast<double>(i) * 2.0, lateral_offset);
    point.longitudinal_velocity_mps = 2.0;
    point.time_from_start.nanosec = static_cast<uint32_t>(i * 100000000UL);
    trajectory.points.push_back(point);
  }

  return trajectory;
}

lanelet::Lanelet make_road_lanelet(const lanelet::Id id, const double y_min, const double y_max)
{
  lanelet::LineString3d left_bound{
    id * 10 + 1,
    {lanelet::Point3d{id * 100 + 1, -5.0, y_max, 0.0},
     lanelet::Point3d{id * 100 + 2, 12.0, y_max, 0.0}}};
  lanelet::LineString3d right_bound{
    id * 10 + 2,
    {lanelet::Point3d{id * 100 + 3, -5.0, y_min, 0.0},
     lanelet::Point3d{id * 100 + 4, 12.0, y_min, 0.0}}};
  lanelet::Lanelet lanelet{id, left_bound, right_bound};
  lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
  return lanelet;
}

lanelet::Lanelet make_shoulder_lanelet(const lanelet::Id id, const double y_min, const double y_max)
{
  auto lanelet = make_road_lanelet(id, y_min, y_max);
  lanelet.setAttribute(lanelet::AttributeName::Subtype, "road_shoulder");
  return lanelet;
}

lanelet::Polygon3d make_hatched_road_marking(
  const lanelet::Id id, const double y_min, const double y_max)
{
  lanelet::Polygon3d polygon{
    id,
    {lanelet::Point3d{id * 100 + 1, -5.0, y_min, 0.0},
     lanelet::Point3d{id * 100 + 2, 12.0, y_min, 0.0},
     lanelet::Point3d{id * 100 + 3, 12.0, y_max, 0.0},
     lanelet::Point3d{id * 100 + 4, -5.0, y_max, 0.0}}};
  polygon.setAttribute(lanelet::AttributeName::Type, "hatched_road_markings");
  polygon.setAttribute("area", "yes");
  return polygon;
}

lanelet::LineString3d make_road_border_line(const lanelet::Id id, const double y)
{
  lanelet::LineString3d line_string{
    id,
    {lanelet::Point3d{id * 100 + 1, -5.0, y, 0.0}, lanelet::Point3d{id * 100 + 2, 12.0, y, 0.0}}};
  line_string.setAttribute(lanelet::AttributeName::Type, "road_border");
  return line_string;
}

std::shared_ptr<RouteHandler> make_route_handler(
  const lanelet::Lanelets & lanelets, const std::vector<lanelet::Polygon3d> & polygons = {},
  const std::vector<lanelet::LineString3d> & line_strings = {})
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::ConstLanelets const_lanelets;
  for (const auto & lanelet : lanelets) {
    map->add(lanelet);
    const_lanelets.push_back(lanelet);
  }
  for (const auto & polygon : polygons) {
    map->add(polygon);
  }
  for (const auto & line_string : line_strings) {
    map->add(line_string);
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg;
  lanelet::utils::conversion::toBinMsg(map, &map_msg);

  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_msg);

  autoware_planning_msgs::msg::LaneletRoute route;
  route.start_pose = make_pose(-4.0, 0.0);
  route.goal_pose = make_pose(11.0, 0.0);
  route.segments = route_handler->createMapSegments(const_lanelets);
  route_handler->setRoute(route);
  return route_handler;
}

autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 2.5, 1.6, 1.0, 1.0, 0.5, 0.5, 1.8, 0.7);
}

}  // namespace

TEST(DrivableAreaComplianceTest, ReturnsOneWhenAllCornersStayInsideDrivableArea)
{
  const auto result = calculate_drivable_area_compliance(
    make_trajectory(0.0), make_route_handler({make_road_lanelet(1, -2.0, 2.0)}),
    make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "compliant");
}

TEST(DrivableAreaComplianceTest, CountsRoadShoulderAsDrivableArea)
{
  const auto result = calculate_drivable_area_compliance(
    make_trajectory(2.2),
    make_route_handler({make_road_lanelet(1, -2.0, 2.0), make_shoulder_lanelet(2, 2.0, 4.0)}),
    make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "compliant");
}

TEST(DrivableAreaComplianceTest, CountsHatchedRoadMarkingAsDrivableArea)
{
  const auto result = calculate_drivable_area_compliance(
    make_trajectory(2.2),
    make_route_handler({make_road_lanelet(1, -2.0, 2.0)}, {make_hatched_road_marking(2, 2.0, 4.0)}),
    make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "compliant");
}

TEST(DrivableAreaComplianceTest, CountsRoadBorderSideTestAsFallbackDrivableArea)
{
  const auto result = calculate_drivable_area_compliance(
    make_trajectory(3.15),
    make_route_handler({make_road_lanelet(1, -2.0, 4.35)}, {}, {make_road_border_line(2, 4.55)}),
    make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "compliant");
}

TEST(DrivableAreaComplianceTest, ReturnsZeroWhenAnyCornerLeavesDrivableArea)
{
  const auto result = calculate_drivable_area_compliance(
    make_trajectory(1.5), make_route_handler({make_road_lanelet(1, -2.0, 2.0)}),
    make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "non_compliant_corner_outside_drivable_area");
  EXPECT_TRUE(std::isfinite(result.debug_info.first_failure_time_s));
  EXPECT_FALSE(result.debug_info.failing_corners.empty());
  EXPECT_FALSE(result.debug_info.ego_horizon_footprints.empty());
  EXPECT_FALSE(result.debug_info.admissible_road_areas.empty());
}

TEST(DrivableAreaComplianceTest, ReturnsUnavailableWhenRequiredInputsAreMissing)
{
  const auto result = calculate_drivable_area_compliance(
    make_trajectory(0.0), std::shared_ptr<RouteHandler>{},
    autoware::vehicle_info_utils::VehicleInfo{});

  EXPECT_FALSE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "unavailable_no_route_handler");
}

}  // namespace autoware::planning_data_analyzer::metrics
