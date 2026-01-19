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
#include <functional>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::planning_debug_tools
{

struct PerceptionReplayerCommonParam
{
  std::string rosbag_path;
  std::string rosbag_format;
  bool tracked_object;
  std::vector<std::string> reference_image_topics;  // Topics for reference images
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
   * @param apply_noise Whether to apply perception noise to objects (default: false)
   */
  void publish_topics_at_timestamp(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp,
    const bool apply_noise = false);

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

  /**
   * @brief Publish reference images at the given timestamp
   * @param bag_timestamp
   * @param current_timestamp
   */
  void publish_reference_images_at_timestamp(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp);

protected:
  const PerceptionReplayerCommonParam param_;

  // Template function to apply perception noise to objects
  template <typename T>
  void apply_perception_noise(
    T & msg, const double update_rate = 0.03, const double x_noise_std = 0.1,
    const double y_noise_std = 0.05)
  {
    if (uniform_dist_(gen_) < update_rate) {
      noise_cache_.clear();
    }

    for (auto & object : msg.objects) {
      const UUID & object_uuid = object.object_id;

      auto it = noise_cache_.find(object_uuid);
      if (it == noise_cache_.end()) {
        it = noise_cache_
               .emplace(
                 object_uuid,
                 std::make_pair(
                   standard_dist_(gen_) * x_noise_std, standard_dist_(gen_) * y_noise_std))
               .first;
      }
      const double noise_x = it->second.first;
      const double noise_y = it->second.second;

      geometry_msgs::msg::Pose & pose = [&]() -> geometry_msgs::msg::Pose & {
        if constexpr (std::is_same_v<T, PredictedObjects>) {
          return object.kinematics.initial_pose_with_covariance.pose;
        } else {
          return object.kinematics.pose_with_covariance.pose;
        }
      }();

      const double obj_yaw = utils::get_yaw_from_quaternion(pose.orientation);
      const double noise_x_world = noise_x * std::cos(obj_yaw) - noise_y * std::sin(obj_yaw);
      const double noise_y_world = noise_x * std::sin(obj_yaw) + noise_y * std::cos(obj_yaw);
      pose.position.x += noise_x_world;
      pose.position.y += noise_y_world;
    }
  }

  // noise
  mutable std::random_device rd_;
  mutable std::mt19937 gen_;
  mutable std::uniform_real_distribution<double> uniform_dist_;
  mutable std::normal_distribution<double> standard_dist_;

  struct UUIDHash
  {
    std::size_t operator()(const UUID & uuid) const
    {
      return *reinterpret_cast<const std::size_t *>(uuid.uuid.data());
    }
  };
  struct UUIDEqual
  {
    bool operator()(const UUID & lhs, const UUID & rhs) const { return lhs.uuid == rhs.uuid; }
  };
  mutable std::unordered_map<UUID, std::pair<double, double>, UUIDHash, UUIDEqual> noise_cache_;

  // rosbag data
  std::vector<utils::DataStamped<Odometry>> rosbag_ego_odom_data_;
  std::vector<utils::DataStamped<PredictedObjects>> rosbag_predicted_objects_data_;
  std::vector<utils::DataStamped<TrackedObjects>> rosbag_tracked_objects_data_;
  std::vector<utils::DataStamped<TrafficLightGroupArray>> rosbag_traffic_signals_data_;
  std::vector<utils::DataStamped<OccupancyGrid>> rosbag_occupancy_grid_data_;

  // Reference image data: topic name -> timestamped messages
  std::unordered_map<std::string, std::vector<utils::DataStamped<CompressedImage>>>
    rosbag_reference_image_data_;

  // load rosbag
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

  // timers
  rclcpp::CallbackGroup::SharedPtr callback_group_check_perception_;
  rclcpp::TimerBase::SharedPtr timer_check_perception_process_;

  // publisher
  rclcpp::PublisherBase::SharedPtr objects_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_signals_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr recorded_ego_as_initialpose_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_as_mission_planning_goal_pub_;

  rclcpp::Publisher<Odometry>::SharedPtr recorded_ego_pub_;

  // Reference image publishers: topic name -> publisher
  std::unordered_map<std::string, rclcpp::Publisher<CompressedImage>::SharedPtr>
    reference_image_pubs_;
};

}  // namespace autoware::planning_debug_tools

#endif  // PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_COMMON_HPP_
