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

#ifndef PERCEPTION_REPLAYER__PERCEPTION_REPRODUCER_HPP_
#define PERCEPTION_REPLAYER__PERCEPTION_REPRODUCER_HPP_

#include "perception_replayer_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>
#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

namespace autoware::planning_debug_tools
{

struct PerceptionReproducerParam : public PerceptionReplayerCommonParam
{
  double search_radius;
  double reproduce_cool_down;
  bool noise;
  bool verbose;
  bool publish_route;
};

class PerceptionReproducer : public PerceptionReplayerCommon
{
public:
  explicit PerceptionReproducer(
    const PerceptionReproducerParam & param, const rclcpp::NodeOptions & node_options);

private:
  void on_timer();
  void on_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // find nearest ego odom index by position
  size_t find_nearest_ego_odom_index(const geometry_msgs::msg::Pose & ego_pose) const;

  // find nearby ego odom indices within search radius
  std::vector<size_t> find_nearby_ego_odom_indices(
    const std::vector<geometry_msgs::msg::Pose> & ego_poses, const double search_radius) const;

  void publish_topics_at_timestamp_with_coordinate_conversion(
    const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp,
    const Odometry & current_ego_odom, const bool apply_noise);

  template <typename T>
  T add_perception_noise(
    T msg, const double update_rate = 0.03, const double x_noise_std = 0.1,
    const double y_noise_std = 0.05)
  {
    if (uniform_dist_(gen_) < update_rate) {
      for (auto & object : msg.objects) {
        const double noise_x = standard_dist_(gen_) * x_noise_std;
        const double noise_y = standard_dist_(gen_) * y_noise_std;
        if constexpr (std::is_same_v<decltype(msg), PredictedObjects>) {
          object.kinematics.initial_pose_with_covariance.pose.position.x += noise_x;
          object.kinematics.initial_pose_with_covariance.pose.position.y += noise_y;
        } else {
          object.kinematics.pose_with_covariance.pose.position.x += noise_x;
          object.kinematics.pose_with_covariance.pose.position.y += noise_y;
        }
      }
    }

    return msg;
  }

private:
  // parameters
  const PerceptionReproducerParam param_;
  double ego_odom_search_radius_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pos_;

  // state management
  std::deque<size_t> reproduce_sequence_indices_;
  std::deque<size_t> cool_down_indices_;
  std::unordered_map<size_t, rclcpp::Time> ego_odom_id2last_published_timestamp_;
  std::optional<geometry_msgs::msg::Pose> last_sequenced_ego_pose_;
  std::optional<rclcpp::Time> last_published_timestamp_;

  // random engine for noise
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
  std::normal_distribution<double> standard_dist_{0.0, 1.0};
};

}  // namespace autoware::planning_debug_tools

#endif  // PERCEPTION_REPLAYER__PERCEPTION_REPRODUCER_HPP_
