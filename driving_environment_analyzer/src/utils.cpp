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

#include "driving_environment_analyzer/utils.hpp"

#include "motion_utils/trajectory/trajectory.hpp"

#include <lanelet2_extension/regulatory_elements/Forward.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>

#include <memory>
#include <string>
#include <vector>

namespace driving_environment_analyzer::utils
{
namespace
{
std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}
}  // namespace

template <class T>
std::vector<double> calcElevationAngle(const T & points)
{
  std::vector<double> elevation_vec(points.size(), 0.0);
  if (points.size() < 2) {
    return elevation_vec;
  }

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto p1 = autoware_universe_utils::getPoint(points.at(i));
    const auto p2 = autoware_universe_utils::getPoint(points.at(i + 1));
    elevation_vec.at(i) = autoware_universe_utils::calcElevationAngle(p1, p2);
  }
  elevation_vec.at(elevation_vec.size() - 1) = elevation_vec.at(elevation_vec.size() - 2);

  return elevation_vec;
}

double calcElevationAngle(const lanelet::ConstLanelet & lane, const Pose & pose)
{
  const auto to_ros_msg = [](const auto & line) {
    std::vector<geometry_msgs::msg::Point> points;
    std::for_each(line.begin(), line.end(), [&points](const auto & p) {
      points.push_back(lanelet::utils::conversion::toGeomMsgPt(p));
    });
    return points;
  };

  const auto points = to_ros_msg(lane.centerline());

  if (points.size() < 2) {
    return 0.0;
  }

  const size_t idx = motion_utils::findNearestSegmentIndex(points, pose.position);

  const auto p1 = autoware_universe_utils::getPoint(points.at(idx));
  const auto p2 = autoware_universe_utils::getPoint(points.at(idx + 1));
  return autoware_universe_utils::calcElevationAngle(p1, p2);
}

double getLaneWidth(const lanelet::ConstLanelet & lane)
{
  const auto to_ros_msg = [](const auto & line) {
    std::vector<geometry_msgs::msg::Point> points;
    std::for_each(line.begin(), line.end(), [&points](const auto & p) {
      points.push_back(lanelet::utils::conversion::toGeomMsgPt(p));
    });
    return points;
  };

  const auto lon_length = motion_utils::calcArcLength(to_ros_msg(lane.centerline()));
  return boost::geometry::area(lane.polygon2d().basicPolygon()) / lon_length;
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
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler)
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
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler)
{
  double value = 0.0;

  for (const auto & lane : lanes) {
    if (existSameDirectionLane(lane, route_handler)) {
      value += lanelet::utils::getLaneletLength3d(lane);
    }
  }

  return value;
}

double getRouteLengthWithOppositeDirectionLane(
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler)
{
  double value = 0.0;

  for (const auto & lane : lanes) {
    if (existOppositeDirectionLane(lane, route_handler)) {
      value += lanelet::utils::getLaneletLength3d(lane);
    }
  }

  return value;
}

double getRouteLengthWithNoAdjacentLane(
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler)
{
  double value = 0.0;

  for (const auto & lane : lanes) {
    if (existSameDirectionLane(lane, route_handler)) {
      continue;
    }

    if (existOppositeDirectionLane(lane, route_handler)) {
      continue;
    }

    value += lanelet::utils::getLaneletLength3d(lane);
  }

  return value;
}

bool existSameDirectionLane(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  const auto right_lanelet = route_handler.getRightLanelet(lane, true, false);
  const auto left_lanelet = route_handler.getLeftLanelet(lane, true, false);
  return right_lanelet.has_value() || left_lanelet.has_value();
}

bool existOppositeDirectionLane(
  const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  {
    const auto lanelet = route_handler.getMostRightLanelet(lane, true, false);
    if (!route_handler.getRightOppositeLanelets(lanelet).empty()) {
      return true;
    }
  }

  {
    const auto lanelet = route_handler.getMostLeftLanelet(lane, true, false);
    if (!route_handler.getRightOppositeLanelets(lanelet).empty()) {
      return true;
    }
  }

  return false;
}

bool existRoadShoulderLane(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  {
    const auto lanelet = route_handler.getMostRightLanelet(lane, true, true);
    const lanelet::Attribute sub_type = lanelet.attribute(lanelet::AttributeName::Subtype);
    if (sub_type.value() == "road_shoulder") {
      return true;
    }
  }

  {
    const auto lanelet = route_handler.getMostLeftLanelet(lane, true, true);
    const lanelet::Attribute sub_type = lanelet.attribute(lanelet::AttributeName::Subtype);
    if (sub_type.value() == "road_shoulder") {
      return true;
    }
  }

  return false;
}

int getLeftLaneletNum(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  const auto left_lanelet = route_handler.getLeftLanelet(lane, true, false);
  if (left_lanelet.has_value()) {
    return getLeftLaneletNum(left_lanelet.value(), route_handler) + 1;
  }

  const auto opposite_lanelets = route_handler.getLeftOppositeLanelets(lane);
  if (!opposite_lanelets.empty()) {
    return getRightLaneletNum(opposite_lanelets.front(), route_handler) + 1;
  }

  return 0;
}

int getRightLaneletNum(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  const auto right_lanelet = route_handler.getRightLanelet(lane, true, false);
  if (right_lanelet.has_value()) {
    return getRightLaneletNum(right_lanelet.value(), route_handler) + 1;
  }

  const auto opposite_lanelets = route_handler.getRightOppositeLanelets(lane);
  if (!opposite_lanelets.empty()) {
    return getLeftLaneletNum(opposite_lanelets.front(), route_handler) + 1;
  }

  return 0;
}

int getTotalLaneletNum(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  return getRightLaneletNum(lane, route_handler) + getLeftLaneletNum(lane, route_handler) + 1;
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

bool existCrosswalk(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler)
{
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto overall_graphs = route_handler.getOverallGraphPtr();
  return !overall_graphs->conflictingInGraph(lane, PEDESTRIAN_GRAPH_ID).empty();
}

bool existCrosswalk(const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler)
{
  for (const auto & lane : lanes) {
    if (existCrosswalk(lane, route_handler)) {
      return true;
    }
  }

  return false;
}

bool isStraightLane(const lanelet::ConstLanelet & lane)
{
  constexpr double THRESHOLD = 50.0;  // radius [m]
  return 1.0 / getMaxCurvature({lane}) > THRESHOLD;
}

size_t getObjectNumber(const PredictedObjects & objects, const std::uint8_t target_class)
{
  return std::count_if(
    objects.objects.begin(), objects.objects.end(), [&target_class](const auto & o) {
      return getHighestProbLabel(o.classification) == target_class;
    });
}

std::string getLaneShape(const lanelet::ConstLanelet & lane)
{
  if (isStraightLane(lane)) {
    return "STRAIGHT";
  }

  return "CURVE";
}

std::string getModuleStatus(const CooperateStatusArray & status_array, const uint8_t module_type)
{
  const auto itr = std::find_if(
    status_array.statuses.begin(), status_array.statuses.end(),
    [&module_type](const auto & s) { return s.module.type == module_type; });

  if (itr == status_array.statuses.end()) {
    return "NONE";
  }

  const auto to_string = [](const auto & command_type) {
    return command_type == Command::DEACTIVATE ? "deactivate" : "activate";
  };

  std::ostringstream ss;
  ss << std::boolalpha;
  ss << "SAFE: " << itr->safe << " COMMAND: " << to_string(itr->command_status.type);
  return ss.str();
}

std::string getEgoBehavior(
  const lanelet::ConstLanelet & lane, [[maybe_unused]] const RouteHandler & route_handler,
  [[maybe_unused]] const Pose & pose)
{
  const std::string turn_direction = lane.attributeOr("turn_direction", "else");

  if (turn_direction == "straight") {
    return "GO STRAIGHT";
  }

  if (turn_direction == "right") {
    return "TURN RIGHT";
  }

  if (turn_direction == "left") {
    return "TURN LEFT";
  }

  return "NONE";
}
}  // namespace driving_environment_analyzer::utils
