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

#ifndef METRICS__EPDMS__DEBUG__EPDMS_DEBUG_WRITER_HPP_
#define METRICS__EPDMS__DEBUG__EPDMS_DEBUG_WRITER_HPP_

#include "metrics/trajectory_metrics.hpp"

#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct EpdmsDebugEnabledMetrics
{
  bool history_comfort{false};
  bool extended_comfort{false};
  bool time_to_collision_within_bound{false};
  bool lane_keeping{false};
  bool ego_progress{false};
  bool drivable_area_compliance{false};
  bool no_at_fault_collision{false};
  bool driving_direction_compliance{false};
  bool traffic_light_compliance{false};
};

void add_epdms_debug_result_topics(
  std::vector<std::pair<std::string, std::string>> & topics,
  const EpdmsDebugEnabledMetrics & enabled);

void write_epdms_point_debug_topics_to_bag(
  const TrajectoryPointMetrics & metrics, const EpdmsDebugEnabledMetrics & enabled,
  rosbag2_cpp::Writer & bag_writer, const rclcpp::Time & timestamp);

void write_epdms_extended_comfort_debug_topics_to_bag(
  const std::string & summary, const std::vector<double> & sample_times,
  const std::vector<double> & delta_acceleration, const std::vector<double> & delta_jerk,
  const std::vector<double> & delta_yaw_rate, const std::vector<double> & delta_yaw_accel,
  rosbag2_cpp::Writer & bag_writer, const rclcpp::Time & timestamp);

void write_epdms_trajectory_horizon_debug_topics_to_bag(
  const autoware_planning_msgs::msg::Trajectory & planned_trajectory,
  const autoware_planning_msgs::msg::Trajectory & ground_truth_trajectory,
  rosbag2_cpp::Writer & bag_writer, const rclcpp::Time & timestamp);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__DEBUG__EPDMS_DEBUG_WRITER_HPP_
