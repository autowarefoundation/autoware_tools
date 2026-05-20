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

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>

namespace
{

using autoware::route_handler::RouteHandler;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

autoware_planning_msgs::msg::Trajectory make_trajectory_crossing_stop_line()
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose = make_pose(3.0, 0.0);
  point.longitudinal_velocity_mps = 2.0;
  trajectory.points.push_back(point);
  return trajectory;
}

autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 2.5, 1.6, 1.0, 1.0, 0.5, 0.5, 1.8, 0.7);
}

std::shared_ptr<RouteHandler> make_route_handler_with_traffic_light(const lanelet::Id signal_id)
{
  lanelet::LineString3d left_bound{
    11, {lanelet::Point3d{101, -5.0, 2.0, 0.0}, lanelet::Point3d{102, 12.0, 2.0, 0.0}}};
  lanelet::LineString3d right_bound{
    12, {lanelet::Point3d{103, -5.0, -2.0, 0.0}, lanelet::Point3d{104, 12.0, -2.0, 0.0}}};
  lanelet::Lanelet road_lanelet{1, left_bound, right_bound};
  road_lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);

  const lanelet::LineString3d traffic_light_base{
    21, {lanelet::Point3d{201, 3.0, 3.0, 4.0}, lanelet::Point3d{202, 3.0, 4.0, 4.0}}};
  const lanelet::LineString3d stop_line{
    22, {lanelet::Point3d{203, 3.0, -2.0, 0.0}, lanelet::Point3d{204, 3.0, 2.0, 0.0}}};
  const auto traffic_light = lanelet::autoware::AutowareTrafficLight::make(
    signal_id, lanelet::AttributeMap{}, {traffic_light_base}, stop_line);
  road_lanelet.addRegulatoryElement(traffic_light);

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road_lanelet);

  autoware_map_msgs::msg::LaneletMapBin map_msg;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  lanelet::utils::conversion::toBinMsg(map, &map_msg);
#pragma GCC diagnostic pop

  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_msg);

  autoware_planning_msgs::msg::LaneletRoute route;
  route.start_pose = make_pose(-4.0, 0.0);
  route.goal_pose = make_pose(11.0, 0.0);
  route.segments = route_handler->createMapSegments({road_lanelet});
  route_handler->setRoute(route);
  return route_handler;
}

std::shared_ptr<TrafficLightGroupArray> make_red_traffic_signals(const lanelet::Id signal_id)
{
  auto traffic_signals = std::make_shared<TrafficLightGroupArray>();
  TrafficLightElement element;
  element.color = TrafficLightElement::RED;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;

  TrafficLightGroup group;
  group.traffic_light_group_id = signal_id;
  group.elements.push_back(element);
  traffic_signals->traffic_light_groups.push_back(group);
  return traffic_signals;
}

}  // namespace

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

TEST(TrafficLightComplianceTest, ReturnsZeroWhenCrossingRedLightStopLine)
{
  constexpr lanelet::Id signal_id = 1001;
  const auto result = autoware::planning_data_analyzer::metrics::calculate_traffic_light_compliance(
    make_trajectory_crossing_stop_line(), make_red_traffic_signals(signal_id),
    make_route_handler_with_traffic_light(signal_id), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "red_light_stop_line_crossed");
}
