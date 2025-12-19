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

#include "perception_reproducer.hpp"

#include "utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace autoware::planning_debug_tools
{

PerceptionReproducer::PerceptionReproducer(
  const PerceptionReproducerParam & param, const rclcpp::NodeOptions & node_options)
: PerceptionReplayerCommon(param, "perception_reproducer", node_options), param_(param)
{
  RCLCPP_INFO(get_logger(), "Starting PerceptionReproducer initialization");

  ego_odom_search_radius_ = param_.search_radius;

  // subscription for /initialpose to refresh cool down
  sub_init_pos_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1,
    std::bind(&PerceptionReproducer::on_initialpose, this, std::placeholders::_1));

  // calculate average ego odom interval
  RCLCPP_INFO(get_logger(), "Calculating average ego odom interval");
  double total_interval = 0.0;
  for (size_t i = 1; i < rosbag_ego_odom_data_.size(); ++i) {
    const auto interval =
      (rosbag_ego_odom_data_[i].first - rosbag_ego_odom_data_[i - 1].first).seconds();
    total_interval += interval;
  }
  const double average_ego_odom_interval =
    total_interval / static_cast<double>(rosbag_ego_odom_data_.size() - 1);
  RCLCPP_INFO(get_logger(), "Average ego odom interval: %.3f seconds", average_ego_odom_interval);

  // start main timer callback
  RCLCPP_INFO(
    get_logger(), "Creating main timer with interval %.3f seconds", average_ego_odom_interval);
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(average_ego_odom_interval),
    std::bind(&PerceptionReproducer::on_timer, this), callback_group_check_perception_);

  if (param_.publish_route) {
    publish_recorded_ego_pose(get_bag_start_time());
    // temporarily add a sleep because sometimes the route is not generated correctly without it.
    // Need to consider a proper solution.
    rclcpp::sleep_for(std::chrono::seconds(2));
    publish_goal_pose();
  }

  RCLCPP_INFO(get_logger(), "PerceptionReproducer initialization completed");
}

void PerceptionReproducer::on_initialpose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  (void)msg;
  cool_down_indices_.clear();

  last_sequenced_ego_pose_.reset();

  if (!last_published_timestamp_.has_value()) {
    const auto nearest_ego_odom_idx = find_nearest_ego_odom_index(msg->pose.pose);
    last_published_timestamp_ = rosbag_ego_odom_data_[nearest_ego_odom_idx].first;
  }

  RCLCPP_INFO(get_logger(), "Cool down indices and last sequenced pose cleared by /initialpose");
}

void PerceptionReproducer::on_timer()
{
  const auto timer_start = std::chrono::high_resolution_clock::now();
  const auto current_timestamp = this->get_clock()->now();

  // check if ego_odom is available
  const auto ego_odom_opt = get_latest_ego_odom();
  if (!ego_odom_opt.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No ego odom found.");
    return;
  }

  const auto ego_odom = ego_odom_opt.value();
  const auto ego_pose = ego_odom.pose.pose;

  // calculate distance moved since last sequenced ego pose
  const double dist_moved =
    last_sequenced_ego_pose_.has_value()
      ? utils::calculate_distance_2d(ego_pose.position, last_sequenced_ego_pose_->position)
      : 999.0;

  // update the reproduce sequence if the distance moved is greater than the search radius
  if (dist_moved > ego_odom_search_radius_) {
    last_sequenced_ego_pose_ = ego_pose;

    // find nearby ego odom indices
    const auto nearest_ego_odom_idx = find_nearest_ego_odom_index(ego_pose);
    std::vector<geometry_msgs::msg::Pose> nearby_ego_odom_poses;

    const auto nearby_indices_initial =
      find_nearby_ego_odom_indices({ego_pose}, ego_odom_search_radius_);
    if (!nearby_indices_initial.empty()) {
      for (const auto idx : nearby_indices_initial) {
        nearby_ego_odom_poses.push_back(rosbag_ego_odom_data_[idx].second.pose.pose);
      }
    } else {
      nearby_ego_odom_poses.push_back(rosbag_ego_odom_data_[nearest_ego_odom_idx].second.pose.pose);
    }

    // find a list of ego odom around the nearest ego odom position
    auto ego_odom_indices =
      find_nearby_ego_odom_indices(nearby_ego_odom_poses, param_.search_radius);

    // remove indices that are in cool down list
    while (!cool_down_indices_.empty()) {
      const auto last_timestamp = ego_odom_id2last_published_timestamp_[cool_down_indices_.front()];
      if ((current_timestamp - last_timestamp).seconds() > param_.reproduce_cool_down) {
        cool_down_indices_.pop_front();
      } else {
        break;
      }
    }

    // update reproduce_sequence with data not in cool down list
    ego_odom_indices.erase(
      std::remove_if(
        ego_odom_indices.begin(), ego_odom_indices.end(),
        [this](const size_t idx) {
          return std::find(cool_down_indices_.begin(), cool_down_indices_.end(), idx) !=
                 cool_down_indices_.end();
        }),
      ego_odom_indices.end());

    std::sort(ego_odom_indices.begin(), ego_odom_indices.end());
    reproduce_sequence_indices_ =
      std::deque<size_t>(ego_odom_indices.begin(), ego_odom_indices.end());
  }

  if (param_.verbose) {
    std::string indices_str;
    for (size_t i = 0; i < std::min<size_t>(20, reproduce_sequence_indices_.size()); ++i) {
      indices_str += std::to_string(reproduce_sequence_indices_[i]) + " ";
    }
    RCLCPP_INFO(get_logger(), "reproduce_sequence_indices: %s", indices_str.c_str());
  }

  // determine whether to repeat previous message
  bool repeat_flag = reproduce_sequence_indices_.empty();

  // add constraint to avoid publishing too fast when speed gap exists
  if (!repeat_flag) {
    const double ego_speed = utils::calculate_speed_2d(ego_odom.twist.twist);

    const size_t ego_odom_idx = reproduce_sequence_indices_.front();
    const auto & ego_odom_msg = rosbag_ego_odom_data_[ego_odom_idx].second;
    const double ego_rosbag_speed = utils::calculate_speed_2d(ego_odom_msg.twist.twist);

    const double ego_rosbag_dist =
      utils::calculate_distance_2d(ego_pose.position, ego_odom_msg.pose.pose.position);

    repeat_flag = (ego_rosbag_speed > ego_speed * 2.0) && (ego_rosbag_speed > 3.0) &&
                  (ego_rosbag_dist > ego_odom_search_radius_);
  }

  // publish messages
  const auto bag_timestamp = [&]() -> std::optional<rclcpp::Time> {
    if (!repeat_flag) {
      const size_t ego_odom_idx = reproduce_sequence_indices_.front();
      reproduce_sequence_indices_.pop_front();

      const auto pose_timestamp = rosbag_ego_odom_data_[ego_odom_idx].first;

      // save the timestamp for potential repeat
      last_published_timestamp_ = pose_timestamp;

      // update cool down info
      ego_odom_id2last_published_timestamp_[ego_odom_idx] = current_timestamp;
      cool_down_indices_.push_back(ego_odom_idx);

      return pose_timestamp;
    }

    return last_published_timestamp_;
  }();

  if (bag_timestamp.has_value()) {
    publish_topics_at_timestamp(
      bag_timestamp.value(), current_timestamp, param_.noise && repeat_flag);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No valid bag timestamp to publish.");
  }

  const auto timer_end = std::chrono::high_resolution_clock::now();
  const auto total_time =
    std::chrono::duration_cast<std::chrono::microseconds>(timer_end - timer_start).count() / 1000.0;

  if (param_.verbose) {
    RCLCPP_INFO(get_logger(), "on_timer processing time: %.3f ms", total_time);
  }
}

size_t PerceptionReproducer::find_nearest_ego_odom_index(
  const geometry_msgs::msg::Pose & ego_pose) const
{
  const double target_x = ego_pose.position.x;
  const double target_y = ego_pose.position.y;

  double min_dist_squared = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;

  for (size_t i = 0; i < rosbag_ego_odom_data_.size(); ++i) {
    const auto & odom_pos = rosbag_ego_odom_data_[i].second.pose.pose.position;
    const double dx = odom_pos.x - target_x;
    const double dy = odom_pos.y - target_y;
    const double dist_squared = dx * dx + dy * dy;

    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

std::vector<size_t> PerceptionReproducer::find_nearby_ego_odom_indices(
  const std::vector<geometry_msgs::msg::Pose> & ego_poses, const double search_radius) const
{
  const double search_radius_squared = search_radius * search_radius;
  std::vector<size_t> nearby_indices;

  for (size_t i = 0; i < rosbag_ego_odom_data_.size(); ++i) {
    const auto & odom_pos = rosbag_ego_odom_data_[i].second.pose.pose.position;

    for (const auto & ego_pose : ego_poses) {
      const double dx = odom_pos.x - ego_pose.position.x;
      const double dy = odom_pos.y - ego_pose.position.y;
      const double dist_squared = dx * dx + dy * dy;

      if (dist_squared <= search_radius_squared) {
        nearby_indices.push_back(i);
        break;  // found within radius, no need to check other ego_poses
      }
    }
  }

  return nearby_indices;
}

}  // namespace autoware::planning_debug_tools
