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

#ifndef UTILS__OVERRIDE_WINDOWS_HPP_
#define UTILS__OVERRIDE_WINDOWS_HPP_

#include <rclcpp/time.hpp>

#include <rcutils/time.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::utils
{

using ControlModeEvent = std::pair<rcutils_time_point_value_t, uint8_t>;
using OverrideWindow = std::pair<rclcpp::Time, rclcpp::Time>;

/**
 * @brief Derive [start, end] override windows from a sorted ControlModeReport timeline.
 *
 * A window is opened at every transition from AUTONOMOUS to MANUAL and closes
 * window_sec seconds after the transition. Events must be sorted by timestamp
 * ascending; ties are handled in input order (the predecessor sample defines
 * the previous mode). Non-AUTONOMOUS->MANUAL transitions are ignored.
 *
 * @param events Sorted (timestamp_ns, mode) timeline.
 * @param window_sec Window duration in seconds. A non-positive value returns an
 *                   empty list.
 * @return Override windows in event order. End time is start + window_sec and
 *         treated as inclusive by is_within_any_window().
 */
std::vector<OverrideWindow> compute_override_windows(
  const std::vector<ControlModeEvent> & events, double window_sec);

/**
 * @brief Test whether @p t lies within any of @p windows.
 *
 * Both ends are inclusive so a sample exactly window_sec after the transition
 * still counts as inside the window.
 */
bool is_within_any_window(const rclcpp::Time & t, const std::vector<OverrideWindow> & windows);

}  // namespace autoware::planning_data_analyzer::utils

#endif  // UTILS__OVERRIDE_WINDOWS_HPP_
