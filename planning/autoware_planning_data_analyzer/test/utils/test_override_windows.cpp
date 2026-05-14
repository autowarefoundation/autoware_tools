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

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace
{

using autoware::planning_data_analyzer::utils::ControlModeEvent;
using autoware::planning_data_analyzer::utils::OverrideWindow;
using autoware::planning_data_analyzer::utils::compute_override_windows;
using autoware::planning_data_analyzer::utils::is_within_any_window;

using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

constexpr rcutils_time_point_value_t kNs = 1'000'000'000LL;

ControlModeEvent ev(const double seconds, const uint8_t mode)
{
  return {static_cast<rcutils_time_point_value_t>(seconds * kNs), mode};
}

}  // namespace

TEST(OverrideWindowsTest, ReturnsEmptyForEmptyEvents)
{
  const auto windows = compute_override_windows({}, 1.0);
  EXPECT_TRUE(windows.empty());
}

TEST(OverrideWindowsTest, ReturnsEmptyForSingleEvent)
{
  const std::vector<ControlModeEvent> events{ev(0.0, ControlModeReport::AUTONOMOUS)};
  const auto windows = compute_override_windows(events, 1.0);
  EXPECT_TRUE(windows.empty());
}

TEST(OverrideWindowsTest, ReturnsEmptyForNonPositiveWindow)
{
  const std::vector<ControlModeEvent> events{
    ev(0.0, ControlModeReport::AUTONOMOUS), ev(1.0, ControlModeReport::MANUAL)};
  EXPECT_TRUE(compute_override_windows(events, 0.0).empty());
  EXPECT_TRUE(compute_override_windows(events, -1.0).empty());
}

TEST(OverrideWindowsTest, OpensWindowOnAutonomousToManualTransition)
{
  const std::vector<ControlModeEvent> events{
    ev(0.0, ControlModeReport::AUTONOMOUS), ev(2.0, ControlModeReport::MANUAL)};
  const auto windows = compute_override_windows(events, 0.5);
  ASSERT_EQ(windows.size(), 1u);
  EXPECT_EQ(windows[0].first.nanoseconds(), static_cast<int64_t>(2.0 * kNs));
  EXPECT_EQ(windows[0].second.nanoseconds(), static_cast<int64_t>(2.5 * kNs));
}

TEST(OverrideWindowsTest, IgnoresManualToAutonomousTransition)
{
  const std::vector<ControlModeEvent> events{
    ev(0.0, ControlModeReport::MANUAL), ev(1.0, ControlModeReport::AUTONOMOUS)};
  EXPECT_TRUE(compute_override_windows(events, 1.0).empty());
}

TEST(OverrideWindowsTest, IgnoresAutonomousToNonManualTransition)
{
  const std::vector<ControlModeEvent> events{
    ev(0.0, ControlModeReport::AUTONOMOUS), ev(1.0, ControlModeReport::DISENGAGED),
    ev(2.0, ControlModeReport::AUTONOMOUS_STEER_ONLY)};
  EXPECT_TRUE(compute_override_windows(events, 1.0).empty());
}

TEST(OverrideWindowsTest, DetectsMultipleTransitions)
{
  const std::vector<ControlModeEvent> events{
    ev(0.0, ControlModeReport::AUTONOMOUS),  //
    ev(1.0, ControlModeReport::MANUAL),      // -> window 1 at t=1
    ev(2.0, ControlModeReport::AUTONOMOUS),  //
    ev(3.0, ControlModeReport::MANUAL),      // -> window 2 at t=3
    ev(4.0, ControlModeReport::MANUAL),      // no transition (MANUAL->MANUAL)
  };
  const auto windows = compute_override_windows(events, 0.5);
  ASSERT_EQ(windows.size(), 2u);
  EXPECT_EQ(windows[0].first.nanoseconds(), static_cast<int64_t>(1.0 * kNs));
  EXPECT_EQ(windows[1].first.nanoseconds(), static_cast<int64_t>(3.0 * kNs));
}

TEST(OverrideWindowsTest, IsWithinAnyWindowHonoursBothEnds)
{
  const std::vector<OverrideWindow> windows{
    {rclcpp::Time(static_cast<int64_t>(1.0 * kNs), RCL_ROS_TIME),
     rclcpp::Time(static_cast<int64_t>(2.0 * kNs), RCL_ROS_TIME)}};
  const auto t = [](double s) {
    return rclcpp::Time(static_cast<int64_t>(s * kNs), RCL_ROS_TIME);
  };
  // Inside
  EXPECT_TRUE(is_within_any_window(t(1.5), windows));
  // On boundaries (inclusive)
  EXPECT_TRUE(is_within_any_window(t(1.0), windows));
  EXPECT_TRUE(is_within_any_window(t(2.0), windows));
  // Outside
  EXPECT_FALSE(is_within_any_window(t(0.9), windows));
  EXPECT_FALSE(is_within_any_window(t(2.1), windows));
}

TEST(OverrideWindowsTest, IsWithinAnyWindowReturnsFalseWhenWindowsEmpty)
{
  EXPECT_FALSE(
    is_within_any_window(rclcpp::Time(static_cast<int64_t>(1.0 * kNs), RCL_ROS_TIME), {}));
}

TEST(OverrideWindowsTest, ComputedWindowsUseRosTimeClock)
{
  const std::vector<ControlModeEvent> events{
    ev(0.0, ControlModeReport::AUTONOMOUS), ev(1.0, ControlModeReport::MANUAL)};
  const auto windows = compute_override_windows(events, 0.5);
  ASSERT_EQ(windows.size(), 1u);
  // Windows must be comparable to timestamps derived from ROS message stamps.
  EXPECT_EQ(windows[0].first.get_clock_type(), RCL_ROS_TIME);
  EXPECT_EQ(windows[0].second.get_clock_type(), RCL_ROS_TIME);
}
