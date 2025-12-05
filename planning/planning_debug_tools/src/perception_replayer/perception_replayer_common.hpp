// Copyright 2025 TIER IV, Inc.
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

#ifndef PLANNING_DEBUG_TOOLS__PERCEPTION_REPLAYER_COMMON_HPP_
#define PLANNING_DEBUG_TOOLS__PERCEPTION_REPLAYER_COMMON_HPP_

#include "type_alias.hpp"
#include "utils.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_debug_tools
{

struct PerceptionReplayerCommonParam
{
  std::string rosbag_path;
  std::string rosbag_format;
  bool detected_object;
  bool tracked_object;
};

class PerceptionReplayerCommon : public rclcpp::Node
{
public:
  explicit PerceptionReplayerCommon(
    const PerceptionReplayerCommonParam & param, const std::string & node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief Get the rosbag start time
   * @return rclcpp::Time
   */
  rclcpp::Time get_bag_start_time() const
  {
    if (rosbag_ego_odom_data_.empty()) {
      throw std::runtime_error("No ego odom data available");
    }
    return rosbag_ego_odom_data_.front().first;
  }

  /**
   * @brief Get the rosbag end timestamp
   * @return rclcpp::Time
   */
  rclcpp::Time get_bag_end_timestamp() const
  {
    if (rosbag_ego_odom_data_.empty()) {
      throw std::runtime_error("No ego odom data available");
    }
    return rosbag_ego_odom_data_.back().first;
  }

  /**
   * @brief Publish objects and traffic signalsat the given timestamp
   * @param bag_timestamp
   * @param current_timestamp
   */
  void publish_topics_at_timestamp(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp);

  /**
   * @brief Publish objects and traffic signals at the given timestamp with coordinate conversion
   * @param bag_timestamp
   * @param current_timestamp
   * @param current_ego_odom
   */
  void publish_topics_at_timestamp_with_coordinate_conversion(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp,
    const Odometry & current_ego_odom);

  /**
   * @brief Publish the recorded ego pose for debugging
   * @param bag_timestamp
   */
  void publish_recorded_ego_pose(rclcpp::Time bag_timestamp);

  /**
   * @brief Publish the last recorded ego pose as goal pose
   */
  void publish_goal_pose();

  /**
   * @brief Publish an empty pointcloud at the given timestamp
   * @param current_timestamp
   */
  void publish_empty_pointcloud(const rclcpp::Time & current_timestamp);

  std::optional<Odometry> get_latest_ego_odom() const
  {
    return ego_odom_ ? std::make_optional<Odometry>(*ego_odom_) : std::nullopt;
  }

protected:
  const PerceptionReplayerCommonParam param_;

  // rosbag data
  // TODO(odashima): move to private
  std::vector<utils::DataStamped<Odometry>> rosbag_ego_odom_data_;

private:
  void load_rosbag(const std::string & rosbag_path, const std::string & rosbag_format);
  std::vector<std::string> find_rosbag_files(
    const std::string & directory_path, const std::string & rosbag_format) const;
  Odometry find_ego_odom_by_timestamp(const rclcpp::Time & timestamp) const;

  void publish_traffic_ligihts_at_timestamp(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp);

  // timer callback to periodically check and kill online perception nodes
  void kill_online_perception_node();
  rclcpp::TimerBase::SharedPtr timer_check_perception_process_;

  // subscriber
  void on_ego_odom(const Odometry::SharedPtr msg);
  rclcpp::Subscription<Odometry>::SharedPtr ego_odom_sub_;
  Odometry::SharedPtr ego_odom_;

  // publisher
  rclcpp::PublisherBase::SharedPtr objects_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_signals_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr recorded_ego_as_initialpose_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_as_mission_planning_goal_pub_;

  rclcpp::Publisher<Odometry>::SharedPtr recorded_ego_pub_;

  // rosbag data
  std::vector<utils::DataStamped<PredictedObjects>> rosbag_predicted_objects_data_;
  std::vector<utils::DataStamped<TrackedObjects>> rosbag_tracked_objects_data_;
  std::vector<utils::DataStamped<DetectedObjects>> rosbag_detected_objects_data_;
  std::vector<utils::DataStamped<TrafficLightGroupArray>> rosbag_traffic_signals_data_;
};

}  // namespace autoware::planning_debug_tools

#endif  // PLANNING_DEBUG_TOOLS__PERCEPTION_REPLAYER_COMMON_HPP_