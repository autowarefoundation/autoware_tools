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

#include "utils/override_windows.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>

namespace autoware::planning_data_analyzer::utils
{

std::vector<OverrideWindow> compute_override_windows(
  const std::vector<ControlModeEvent> & events, const double window_sec)
{
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
  std::vector<OverrideWindow> windows;
  if (window_sec <= 0.0 || events.size() < 2u) {
    return windows;
  }

  const auto window_duration = rclcpp::Duration::from_seconds(window_sec);
  auto previous_mode = events.front().second;
  for (std::size_t i = 1; i < events.size(); ++i) {
    const auto current_mode = events[i].second;
    if (
      previous_mode == ControlModeReport::AUTONOMOUS &&
      current_mode == ControlModeReport::MANUAL) {
      // Use RCL_ROS_TIME so windows can be compared against trajectory timestamps
      // produced from ROS message stamps (which default to RCL_ROS_TIME).
      const rclcpp::Time start_time(events[i].first, RCL_ROS_TIME);
      windows.emplace_back(start_time, start_time + window_duration);
    }
    previous_mode = current_mode;
  }
  return windows;
}

bool is_within_any_window(
  const rclcpp::Time & t, const std::vector<OverrideWindow> & windows)
{
  for (const auto & [start, end] : windows) {
    if (t >= start && t <= end) {
      return true;
    }
  }
  return false;
}

}  // namespace autoware::planning_data_analyzer::utils
