// Copyright 2024 TIER IV, Inc.
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

#include "driving_environment_analyzer/node.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "rosbag2_storage/storage_filter.hpp"

#include <lanelet2_extension/regulatory_elements/Forward.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>

#include <memory>
#include <string>
#include <vector>

namespace driving_environment_analyzer
{
namespace
{
template <class T>
std::vector<double> calcElevationAngle(const T & points)
{
  std::vector<double> elevation_vec(points.size(), 0.0);
  if (points.size() < 2) {
    return elevation_vec;
  }

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto p1 = tier4_autoware_utils::getPoint(points.at(i));
    const auto p2 = tier4_autoware_utils::getPoint(points.at(i + 1));
    elevation_vec.at(i) = tier4_autoware_utils::calcElevationAngle(p1, p2);
  }
  elevation_vec.at(elevation_vec.size() - 1) = elevation_vec.at(elevation_vec.size() - 2);

  return elevation_vec;
}

double getRouteLength(const lanelet::ConstLanelets & lanes)
{
  return lanelet::utils::getLaneletLength3d(lanes);
}

double getMaxCurvature(const lanelet::ConstLanelets & lanes)
{
  const auto to_ros_msg = [](const auto & line) {
    std::vector<geometry_msgs::msg::Point> points;
    std::for_each(line.begin(), line.end(), [&points](const auto & p) {
      points.push_back(lanelet::utils::conversion::toGeomMsgPt(p));
    });
    return points;
  };

  double max_value = 0.0;

  for (const auto & lane : lanes) {
    const auto values = motion_utils::calcCurvature(to_ros_msg(lane.centerline()));
    const auto max_value_itr = std::max_element(values.begin(), values.end());
    if (max_value_itr == values.end()) {
      continue;
    }

    if (max_value < *max_value_itr) {
      max_value = *max_value_itr;
    }
  }

  return max_value;
}

std::pair<double, double> getLaneWidth(const lanelet::ConstLanelets & lanes)
{
  const auto to_ros_msg = [](const auto & line) {
    std::vector<geometry_msgs::msg::Point> points;
    std::for_each(line.begin(), line.end(), [&points](const auto & p) {
      points.push_back(lanelet::utils::conversion::toGeomMsgPt(p));
    });
    return points;
  };

  double min_value = std::numeric_limits<double>::max();
  double max_value = 0.0;

  for (const auto & lane : lanes) {
    const auto lon_length = motion_utils::calcArcLength(to_ros_msg(lane.centerline()));
    const auto width = boost::geometry::area(lane.polygon2d().basicPolygon()) / lon_length;

    if (min_value > width) {
      min_value = width;
    }

    if (max_value < width) {
      max_value = width;
    }
  }

  return std::make_pair(min_value, max_value);
}

std::pair<double, double> getElevation(const lanelet::ConstLanelets & lanes)
{
  const auto to_ros_msg = [](const auto & line) {
    std::vector<geometry_msgs::msg::Point> points;
    std::for_each(line.begin(), line.end(), [&points](const auto & p) {
      points.push_back(lanelet::utils::conversion::toGeomMsgPt(p));
    });
    return points;
  };

  double min_value = std::numeric_limits<double>::max();
  double max_value = 0.0;

  for (const auto & lane : lanes) {
    const auto values = calcElevationAngle(to_ros_msg(lane.centerline()));
    const auto max_value_itr = std::max_element(values.begin(), values.end());
    const auto min_value_itr = std::min_element(values.begin(), values.end());

    if (min_value_itr == values.end()) {
      continue;
    }

    if (min_value > *min_value_itr) {
      min_value = *min_value_itr;
    }

    if (max_value_itr == values.end()) {
      continue;
    }

    if (max_value < *max_value_itr) {
      max_value = *max_value_itr;
    }
  }

  return std::make_pair(min_value, max_value);
}

std::pair<double, double> getSpeedLimit(
  const lanelet::ConstLanelets & lanes, const route_handler::RouteHandler & route_handler)
{
  const auto traffic_rule = route_handler.getTrafficRulesPtr();

  double min_value = std::numeric_limits<double>::max();
  double max_value = 0.0;

  for (const auto & lane : lanes) {
    const auto limit = traffic_rule->speedLimit(lane).speedLimit;
    min_value = std::min(limit.value(), min_value);
    max_value = std::max(limit.value(), max_value);
  }

  return std::make_pair(min_value, max_value);
}

double getRouteLengthWithSameDirectionLane(
  const lanelet::ConstLanelets & lanes, const route_handler::RouteHandler & route_handler)
{
  double value = 0.0;

  for (const auto & lane : lanes) {
    const auto right_lanelet = route_handler.getRightLanelet(lane, false, false);
    const auto left_lanelet = route_handler.getLeftLanelet(lane, false, false);
    if (right_lanelet.has_value() || left_lanelet.has_value()) {
      value += lanelet::utils::getLaneletLength3d(lane);
    }
  }

  return value;
}

double getRouteLengthWithOppositeDirectionLane(
  const lanelet::ConstLanelets & lanes, const route_handler::RouteHandler & route_handler)
{
  double value = 0.0;

  for (const auto & lane : lanes) {
    const auto right_lanelet = route_handler.getRightOppositeLanelets(lane);
    const auto left_lanelet = route_handler.getLeftOppositeLanelets(lane);
    if (!right_lanelet.empty() || !left_lanelet.empty()) {
      value += lanelet::utils::getLaneletLength3d(lane);
    }
  }

  return value;
}

double getRouteLengthWithNoAdjacentLane(
  const lanelet::ConstLanelets & lanes, const route_handler::RouteHandler & route_handler)
{
  double value = 0.0;

  for (const auto & lane : lanes) {
    {
      const auto right_lanelet = route_handler.getRightLanelet(lane, false, false);
      const auto left_lanelet = route_handler.getLeftLanelet(lane, false, false);
      if (right_lanelet.has_value() || left_lanelet.has_value()) {
        continue;
      }
    }

    {
      const auto right_lanelet = route_handler.getRightOppositeLanelets(lane);
      const auto left_lanelet = route_handler.getLeftOppositeLanelets(lane);
      if (!right_lanelet.empty() || !left_lanelet.empty()) {
        continue;
      }
    }

    value += lanelet::utils::getLaneletLength3d(lane);
  }

  return value;
}

bool existTrafficLight(const lanelet::ConstLanelets & lanes)
{
  for (const auto & lane : lanes) {
    if (!lane.regulatoryElementsAs<lanelet::TrafficLight>().empty()) {
      return true;
    }
  }

  return false;
}

bool existIntersection(const lanelet::ConstLanelets & lanes)
{
  for (const auto & lane : lanes) {
    std::string turn_direction = lane.attributeOr("turn_direction", "else");
    if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
      return true;
    }
  }

  return false;
}

bool existCrosswalk(
  const lanelet::ConstLanelets & lanes, const route_handler::RouteHandler & route_handler)
{
  const auto overall_graphs = route_handler.getOverallGraphPtr();
  for (const auto & lane : lanes) {
    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    if (!overall_graphs->conflictingInGraph(lane, PEDESTRIAN_GRAPH_ID).empty()) {
      return true;
    }
  }

  return false;
}
}  // namespace

DrivingEnvironmentAnalyzer::DrivingEnvironmentAnalyzer(const rclcpp::NodeOptions & node_options)
: Node("driving_environment_analyzer", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  const auto bag_filename = declare_parameter<std::string>("bag_path");
  const auto use_map_in_bag = declare_parameter<bool>("use_map_in_bag");

  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DrivingEnvironmentAnalyzer::analyze, this));

  sub_map_ = create_subscription<LaneletMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DrivingEnvironmentAnalyzer::onMap, this, _1));

  reader_.open(bag_filename);

  if (!isDataReady(use_map_in_bag)) {
    rclcpp::shutdown();
  }
}

bool DrivingEnvironmentAnalyzer::isDataReady(const bool use_map_in_bag)
{
  const std::string topic_route = "/planning/mission_planning/route";
  const std::string topic_map = "/map/vector_map";

  rclcpp::Serialization<LaneletRoute> serializer_route;
  rclcpp::Serialization<LaneletMapBin> serializer_map;

  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(topic_route);
  filter.topics.emplace_back(topic_map);
  reader_.set_filter(filter);

  while (reader_.has_next()) {
    const auto data = reader_.read_next();

    if (data->topic_name == topic_route) {
      rclcpp::SerializedMessage serialized_msg(*data->serialized_data);
      const auto deserialized_message = std::make_shared<LaneletRoute>();
      serializer_route.deserialize_message(&serialized_msg, deserialized_message.get());
      route_msgs_.push_back(*deserialized_message);
    }

    if (data->topic_name == topic_map) {
      rclcpp::SerializedMessage serialized_msg(*data->serialized_data);
      const auto deserialized_message = std::make_shared<LaneletMapBin>();
      serializer_route.deserialize_message(&serialized_msg, deserialized_message.get());
      route_handler_.setMap(*deserialized_message);
      has_map_data_ = true;
    }
  }

  if (route_msgs_.empty()) {
    RCLCPP_ERROR(get_logger(), "topic: %s is not included. shutdown.", topic_route.c_str());
    return false;
  }

  if (use_map_in_bag && !has_map_data_) {
    RCLCPP_ERROR(get_logger(), "topic: %s is not included. shutdown.", topic_map.c_str());
    return false;
  }

  return true;
}

void DrivingEnvironmentAnalyzer::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  has_map_data_ = true;
}

void DrivingEnvironmentAnalyzer::analyze()
{
  if (!has_map_data_) {
    return;
  }

  route_handler_.setRoute(route_msgs_.front());

  RCLCPP_INFO(get_logger(), "======================================");
  RCLCPP_INFO(get_logger(), " data is ready. start ODD analysis... ");
  RCLCPP_INFO(get_logger(), "======================================");

  const auto preferred_lanes = route_handler_.getPreferredLanelets();
  RCLCPP_INFO(get_logger(), "- Length of total lanes : %.2f [m]", getRouteLength(preferred_lanes));
  RCLCPP_INFO(
    get_logger(), "- Length of lane that has adjacent lane : %.2f [m]",
    getRouteLengthWithSameDirectionLane(preferred_lanes, route_handler_));
  RCLCPP_INFO(
    get_logger(), "- Length of lane that has opposite lane : %.2f [m]",
    getRouteLengthWithOppositeDirectionLane(preferred_lanes, route_handler_));
  RCLCPP_INFO(
    get_logger(), "- Length of lane that has no adjacent lane : %.2f [m]",
    getRouteLengthWithNoAdjacentLane(preferred_lanes, route_handler_));
  const auto [min_width, max_width] = getLaneWidth(preferred_lanes);
  RCLCPP_INFO(
    get_logger(), "- Min lane width: %.2f [m] Max lane width: %.2f [m]", min_width, max_width);
  const auto max_curvature = getMaxCurvature(preferred_lanes);
  RCLCPP_INFO(get_logger(), "- Max curvature: %f [1/m]", max_curvature);
  RCLCPP_INFO(get_logger(), "- Min curve radius: %.2f [m]", 1.0 / max_curvature);
  const auto [min_elevation, max_elevation] = getElevation(preferred_lanes);
  RCLCPP_INFO(
    get_logger(), "- Min elevation angle: %f [rad] Max elevation angle: %f [rad]", min_elevation,
    max_elevation);
  const auto [min_speed_limit, max_speed_limit] = getSpeedLimit(preferred_lanes, route_handler_);
  RCLCPP_INFO(
    get_logger(), "- Min speed limit: %.2f [m/s] Max speed limit: %.2f [m/s]", min_speed_limit,
    max_speed_limit);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "- Exist traffic light: " << std::boolalpha << existTrafficLight(preferred_lanes));
  RCLCPP_INFO_STREAM(
    get_logger(), "- Exist intersection: " << std::boolalpha << existIntersection(preferred_lanes));
  RCLCPP_INFO_STREAM(
    get_logger(),
    "- Exist crosswalk: " << std::boolalpha << existCrosswalk(preferred_lanes, route_handler_));

  RCLCPP_INFO(get_logger(), "======================================");
  RCLCPP_INFO(get_logger(), " complete ODD analysis. shutdown. ");
  RCLCPP_INFO(get_logger(), "======================================");
  rclcpp::shutdown();
}
}  // namespace driving_environment_analyzer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(driving_environment_analyzer::DrivingEnvironmentAnalyzer)
