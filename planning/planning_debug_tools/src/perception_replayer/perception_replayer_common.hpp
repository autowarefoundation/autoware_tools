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

#ifndef PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_COMMON_HPP_
#define PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_COMMON_HPP_

#include "type_alias.hpp"
#include "utils.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
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
   * @brief Publish objects and traffic light the given timestamp
   * @param bag_timestamp
   * @param current_timestamp
   */
  void publish_topics_at_timestamp(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp);

  /**
   * @brief Publish the recorded ego pose for debugging
   * @param bag_timestamp
   */
  void publish_recorded_ego_pose(rclcpp::Time bag_timestamp);

  /**
   * @brief Publish the last recorded ego pose as goal pose
   */
  void publish_goal_pose();

  std::optional<Odometry> get_latest_ego_odom() const
  {
    return ego_odom_ ? std::make_optional<Odometry>(*ego_odom_) : std::nullopt;
  }

  /**
   * @brief Publish traffic light the given timestamp
   * @param bag_timestamp
   * @param current_timestamp
   */
  void publish_traffic_lights_at_timestamp(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp);

protected:
  const PerceptionReplayerCommonParam param_;

  // rosbag data
  std::vector<utils::DataStamped<Odometry>> rosbag_ego_odom_data_;
  std::vector<utils::DataStamped<PredictedObjects>> rosbag_predicted_objects_data_;
  std::vector<utils::DataStamped<TrackedObjects>> rosbag_tracked_objects_data_;
  std::vector<utils::DataStamped<TrafficLightGroupArray>> rosbag_traffic_signals_data_;
  std::vector<utils::DataStamped<OccupancyGrid>> rosbag_occupancy_grid_data_;

  void load_rosbag(const std::string & rosbag_path, const std::string & rosbag_format);
  std::vector<std::string> find_rosbag_files(
    const std::string & directory_path, const std::string & rosbag_format) const;
  Odometry find_ego_odom_by_timestamp(const rclcpp::Time & timestamp) const;

  void kill_online_perception_node();
  void kill_process(const std::string & process_name);
  void unload_component(const std::string & container_name, const std::string & component_name);

  // subscriber
  void on_ego_odom(const Odometry::SharedPtr msg);
  rclcpp::Subscription<Odometry>::SharedPtr ego_odom_sub_;
  Odometry::SharedPtr ego_odom_;

  // publisher
  rclcpp::PublisherBase::SharedPtr objects_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_signals_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr recorded_ego_as_initialpose_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_as_mission_planning_goal_pub_;

  rclcpp::Publisher<Odometry>::SharedPtr recorded_ego_pub_;

  // cache for occupancy grid to avoid republishing the same message
  size_t last_published_occupancy_grid_idx_ = SIZE_MAX;
};

}  // namespace autoware::planning_debug_tools

#endif  // PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_COMMON_HPP_
