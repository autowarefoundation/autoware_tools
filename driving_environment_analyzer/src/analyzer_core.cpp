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

#include "driving_environment_analyzer/analyzer_core.hpp"

#include "driving_environment_analyzer/utils.hpp"

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace driving_environment_analyzer::analyzer_core
{

AnalyzerCore::AnalyzerCore(rclcpp::Node & node) : logger_{node.get_logger()}
{
}

bool AnalyzerCore::isDataReadyForStaticODDAnalysis() const
{
  if (!route_handler_.isMapMsgReady()) {
    RCLCPP_WARN(logger_, "Map data is not ready.");
    return false;
  }

  if (!route_handler_.isHandlerReady()) {
    RCLCPP_WARN(logger_, "Route data is not ready.");
    return false;
  }

  return true;
}

void AnalyzerCore::setBagFile(const std::string & file_name)
{
  reader_.open(file_name);

  const auto opt_route = getLastTopic<LaneletRoute>("/planning/mission_planning/route");
  if (opt_route.has_value()) {
    route_handler_.setRoute(opt_route.value());
  }

  const auto opt_map = getLastTopic<LaneletMapBin>("/map/vector_map");
  if (opt_map.has_value()) {
    route_handler_.setMap(opt_map.value());
  }
}

template <class T>
std::optional<T> AnalyzerCore::getLastTopic(const std::string & topic_name)
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(topic_name);
  reader_.set_filter(filter);

  if (!reader_.has_next()) {
    return std::nullopt;
  }

  rclcpp::Serialization<T> serializer;

  const auto deserialized_message = std::make_shared<T>();
  while (reader_.has_next()) {
    rclcpp::SerializedMessage serialized_msg(*reader_.read_next()->serialized_data);
    serializer.deserialize_message(&serialized_msg, deserialized_message.get());
  }

  return *deserialized_message;
}

template <class T>
std::optional<T> AnalyzerCore::seekTopic(
  const std::string & topic_name, const rcutils_time_point_value_t & timestamp)
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(topic_name);
  reader_.set_filter(filter);

  rclcpp::Serialization<T> serializer;
  reader_.seek(timestamp);

  if (!reader_.has_next()) {
    return std::nullopt;
  }

  rclcpp::SerializedMessage serialized_msg(*reader_.read_next()->serialized_data);
  const auto deserialized_message = std::make_shared<T>();
  serializer.deserialize_message(&serialized_msg, deserialized_message.get());

  return *deserialized_message;
}

std::optional<ODDRawData> AnalyzerCore::getRawData(const rcutils_time_point_value_t & timestamp)
{
  ODDRawData odd_raw_data;

  odd_raw_data.timestamp = timestamp;

  const auto opt_odometry = seekTopic<Odometry>("/localization/kinematic_state", timestamp * 1e9);
  if (!opt_odometry.has_value()) {
    return std::nullopt;
  } else {
    odd_raw_data.odometry = opt_odometry.value();
  }

  const auto opt_objects =
    seekTopic<PredictedObjects>("/perception/object_recognition/objects", timestamp * 1e9);
  if (!opt_objects.has_value()) {
    return std::nullopt;
  } else {
    odd_raw_data.objects = opt_objects.value();
  }

  const auto opt_rtc_status =
    seekTopic<CooperateStatusArray>("/api/external/get/rtc_status", timestamp * 1e9);
  if (!opt_rtc_status.has_value()) {
    return std::nullopt;
  } else {
    odd_raw_data.rtc_status = opt_rtc_status.value();
  }

  const auto opt_tf = seekTopic<TFMessage>("/tf", timestamp * 1e9);
  if (!opt_tf.has_value()) {
    return std::nullopt;
  } else {
    odd_raw_data.tf = opt_tf.value();
  }

  const auto [start_time, end_time] = getBagStartEndTime();
  const auto opt_tf_static = seekTopic<TFMessage>("/tf_static", start_time * 1e9);
  if (!opt_tf_static.has_value()) {
    return std::nullopt;
  } else {
    odd_raw_data.tf_static = opt_tf_static.value();
  }

  return odd_raw_data;
}

void AnalyzerCore::addHeader(std::ofstream & ofs_csv_file) const
{
  ofs_csv_file << "TIME" << ',';
  ofs_csv_file << "EGO [SPEED]" << ',';
  ofs_csv_file << "EGO [ELEVATION ANGLE]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [AVOIDANCE(R)]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [AVOIDANCE(L)]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [LANE_CHANGE(R)]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [LANE_CHANGE(L)]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [START_PLANNER]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [GOAL_PLANNER]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [CROSSWALK]" << ',';
  ofs_csv_file << "EGO BEHAVIOR [INTERSECTION]" << ',';
  ofs_csv_file << "LANE [ID]" << ',';
  ofs_csv_file << "LANE [WIDTH]" << ',';
  ofs_csv_file << "LANE [SHAPE]" << ',';
  ofs_csv_file << "LANE [RIGHT LANE NUM]" << ',';
  ofs_csv_file << "LANE [LEFT LANE NUM]" << ',';
  ofs_csv_file << "LANE [TOTAL LANE NUM]" << ',';
  ofs_csv_file << "LANE [SAME DIRECTION LANE]" << ',';
  ofs_csv_file << "LANE [OPPOSITE DIRECTION LANE]" << ',';
  ofs_csv_file << "LANE [ROAD SHOULDER]" << ',';
  ofs_csv_file << "OBJECT [UNKNOWN]" << ',';
  ofs_csv_file << "OBJECT [CAR]" << ',';
  ofs_csv_file << "OBJECT [TRUCK]" << ',';
  ofs_csv_file << "OBJECT [BUS]" << ',';
  ofs_csv_file << "OBJECT [TRAILER]" << ',';
  ofs_csv_file << "OBJECT [MOTORCYCLE]" << ',';
  ofs_csv_file << "OBJECT [BICYCLE]" << ',';
  ofs_csv_file << "OBJECT [PEDESTRIAN]" << ',';
  ofs_csv_file << std::endl;
}

void AnalyzerCore::analyzeDynamicODDFactor(std::ofstream & ofs_csv_file) const
{
  std::ostringstream ss;
  ss << std::boolalpha << "\n";
  ss << "***********************************************************\n";
  ss << "                   ODD analysis result\n";
  ss << "***********************************************************\n";
  ss << "Type: TIME SPECIFIED\n";

  const auto write = [&ofs_csv_file](const auto & data) {
    ofs_csv_file << data << ',';
    return data;
  };

  char buffer[128];
  auto seconds = static_cast<time_t>(odd_raw_data_.value().timestamp);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&seconds));
  ss << "Time: " << write(buffer) << "\n";
  ss << "\n";
  ss << "\n";

  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler_.getClosestLaneletWithinRoute(getEgoPose(), &closest_lanelet)) {
    return;
  }

  const auto number = [this](const auto & target_class) {
    return utils::getObjectNumber(odd_raw_data_.value().objects, target_class);
  };

  const auto status = [this](const auto & module_type) {
    return utils::getModuleStatus(odd_raw_data_.value().rtc_status, module_type);
  };

  const auto exist_crosswalk = [this, &closest_lanelet]() {
    return utils::existCrosswalk(closest_lanelet, route_handler_) ? "EXIST" : "NONE";
  };

  const auto to_string = [](const bool exist) { return exist ? "EXIST" : "NONE"; };

  ss << "- EGO INFO\n";
  ss << "  [SPEED]                       : " << write(getEgoSpeed()) << " [m/s]\n";
  ss << "  [ELEVATION ANGLE]             : "
     << write(utils::calcElevationAngle(closest_lanelet, getEgoPose())) << " [rad]\n";
  ss << "\n";

  ss << "- EGO BEHAVIOR\n";
  ss << "  [AVOIDANCE(R)]                : " << write(status(Module::AVOIDANCE_RIGHT)) << "\n";
  ss << "  [AVOIDANCE(L)]                : " << write(status(Module::AVOIDANCE_LEFT)) << "\n";
  ss << "  [LANE_CHANGE(R)]              : " << write(status(Module::LANE_CHANGE_RIGHT)) << "\n";
  ss << "  [LANE_CHANGE(L)]              : " << write(status(Module::LANE_CHANGE_LEFT)) << "\n";
  ss << "  [START_PLANNER]               : " << write(status(Module::START_PLANNER)) << "\n";
  ss << "  [GOAL_PLANNER]                : " << write(status(Module::GOAL_PLANNER)) << "\n";
  ss << "  [CROSSWALK]                   : " << write(exist_crosswalk()) << "\n";
  ss << "  [INTERSECTION]                : "
     << write(utils::getEgoBehavior(closest_lanelet, route_handler_, getEgoPose())) << "\n";
  ss << "\n";

  ss << "- LANE INFO\n";
  ss << "  [ID]                          : " << write(closest_lanelet.id()) << "\n";
  ss << "  [WIDTH]                       : " << write(utils::getLaneWidth(closest_lanelet))
     << " [m]\n";
  ss << "  [SHAPE]                       : " << write(utils::getLaneShape(closest_lanelet)) << "\n";
  ss << "  [RIGHT LANE NUM]              : "
     << write(utils::getRightLaneletNum(closest_lanelet, route_handler_)) << "\n";
  ss << "  [LEFT LANE NUM]               : "
     << write(utils::getLeftLaneletNum(closest_lanelet, route_handler_)) << "\n";
  ss << "  [TOTAL LANE NUM]              : "
     << write(utils::getTotalLaneletNum(closest_lanelet, route_handler_)) << "\n";
  ss << "  [SAME DIRECTION LANE]         : "
     << write(to_string(utils::existSameDirectionLane(closest_lanelet, route_handler_))) << "\n";
  ss << "  [OPPOSITE DIRECTION LANE]     : "
     << write(to_string(utils::existOppositeDirectionLane(closest_lanelet, route_handler_)))
     << "\n";
  ss << "  [ROAD SHOULDER]               : "
     << write(to_string(utils::existRoadShoulderLane(closest_lanelet, route_handler_))) << "\n";
  ss << "\n";

  ss << "- SURROUND OBJECT NUM\n";
  ss << "  [UNKNOWN]                     : " << write(number(ObjectClassification::UNKNOWN))
     << "\n";
  ss << "  [CAR]                         : " << write(number(ObjectClassification::CAR)) << "\n";
  ss << "  [TRUCK]                       : " << write(number(ObjectClassification::TRUCK)) << "\n";
  ss << "  [BUS]                         : " << write(number(ObjectClassification::BUS)) << "\n";
  ss << "  [TRAILER]                     : " << write(number(ObjectClassification::TRAILER))
     << "\n";
  ss << "  [MOTORCYCLE]                  : " << write(number(ObjectClassification::MOTORCYCLE))
     << "\n";
  ss << "  [BICYCLE]                     : " << write(number(ObjectClassification::BICYCLE))
     << "\n";
  ss << "  [PEDESTRIAN]                  : " << write(number(ObjectClassification::PEDESTRIAN))
     << "\n";

  ss << "***********************************************************\n";

  ofs_csv_file << std::endl;

  RCLCPP_INFO_STREAM(logger_, ss.str());
}

void AnalyzerCore::analyzeStaticODDFactor() const
{
  std::ostringstream ss;
  ss << std::boolalpha << "\n";
  ss << "***********************************************************\n";
  ss << "                   ODD analysis result\n";
  ss << "***********************************************************\n";
  ss << "Type: ROUTE SPECIFIED\n";
  ss << "\n";
  ss << "\n";

  const auto preferred_lanes = route_handler_.getPreferredLanelets();
  ss << "- ROUTE INFO\n";
  ss << "  total length                      : " << utils::getRouteLength(preferred_lanes)
     << " [m]\n";
  ss << "  exist same direction lane section : "
     << utils::getRouteLengthWithSameDirectionLane(preferred_lanes, route_handler_) << " [m]\n";
  ss << "  exist same opposite lane section  : "
     << utils::getRouteLengthWithOppositeDirectionLane(preferred_lanes, route_handler_) << " [m]\n";
  ss << "  no adjacent lane section          : "
     << utils::getRouteLengthWithNoAdjacentLane(preferred_lanes, route_handler_) << " [m]\n";

  ss << "  exist traffic light               : " << utils::existTrafficLight(preferred_lanes)
     << "\n";
  ss << "  exist intersection                : " << utils::existIntersection(preferred_lanes)
     << "\n";
  ss << "  exist crosswalk                   : "
     << utils::existCrosswalk(preferred_lanes, route_handler_) << "\n";
  ss << "\n";

  const auto [min_width, max_width] = utils::getLaneWidth(preferred_lanes);
  ss << "- LANE WIDTH\n";
  ss << "  max                               : " << max_width << " [m]\n";
  ss << "  min                               : " << min_width << " [m]\n";
  ss << "\n";

  const auto max_curvature = utils::getMaxCurvature(preferred_lanes);
  ss << "- LANE CURVATURE\n";
  ss << "  max                               : " << max_curvature << " [1/m]\n";
  ss << "  max                               : " << 1.0 / max_curvature << " [m]\n";
  ss << "\n";

  const auto [min_elevation, max_elevation] = utils::getElevation(preferred_lanes);
  ss << "- ELEVATION ANGLE\n";
  ss << "  max                               : " << max_elevation << " [rad]\n";
  ss << "  min                               : " << min_elevation << " [rad]\n";
  ss << "\n";

  const auto [min_speed_limit, max_speed_limit] =
    utils::getSpeedLimit(preferred_lanes, route_handler_);
  ss << "- SPEED LIMIT\n";
  ss << "  max                               : " << max_speed_limit << " [m/s]\n";
  ss << "  min                               : " << min_speed_limit << " [m/s]\n";
  ss << "\n";

  ss << "***********************************************************\n";

  RCLCPP_INFO_STREAM(logger_, ss.str());
}

AnalyzerCore::~AnalyzerCore() = default;
}  // namespace driving_environment_analyzer::analyzer_core
