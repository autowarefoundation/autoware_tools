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

#ifndef AUTOWARE_PLANNING_DATA_ANALYZER_NODE_HPP_
#define AUTOWARE_PLANNING_DATA_ANALYZER_NODE_HPP_

#include "bag_handler.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{

using autoware_planning_msgs::msg::Trajectory;

class AutowarePlanningDataAnalyzerNode : public rclcpp::Node
{
public:
  explicit AutowarePlanningDataAnalyzerNode(const rclcpp::NodeOptions & node_options);

  ~AutowarePlanningDataAnalyzerNode() override;

private:
  enum class EvaluationMode { OPEN_LOOP, OR_SCENE };

  void setup_evaluation_bag_writer();
  void run_evaluation();
  void write_map_and_route_markers_to_bag(const rclcpp::Time & reference_time);
  void create_route_markers(visualization_msgs::msg::MarkerArray & marker_array) const;

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr map_marker_;

  mutable std::mutex mutex_;

  mutable rosbag2_cpp::Reader bag_reader_;

  std::unique_ptr<rosbag2_cpp::Writer> evaluation_bag_writer_;

  std::string map_topic_name_;
  std::string map_marker_topic_name_;
  std::string route_topic_name_;
  std::string odometry_topic_name_;
  std::string trajectory_topic_name_;
  std::string objects_topic_name_;
  std::string tf_topic_name_;
  std::string acceleration_topic_name_;
  std::string steering_topic_name_;

  EvaluationMode evaluation_mode_;
  std::string bag_path_;

  rclcpp::TimerBase::SharedPtr map_check_timer_;

  // Store tf_static messages for evaluation
  tf2_msgs::msg::TFMessage tf_static_msgs_;
};
}  // namespace autoware::planning_data_analyzer

#endif  // AUTOWARE_PLANNING_DATA_ANALYZER_NODE_HPP_
