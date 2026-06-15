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

#include "metrics/epdms/subscores/extended_comfort.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

autoware_planning_msgs::msg::Trajectory make_trajectory(
  const std::vector<double> & accelerations, const std::vector<double> & yaws,
  const int32_t stamp_nsec = 0, const std::vector<uint32_t> & sample_times_nsec = {})
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.nanosec = static_cast<uint32_t>(stamp_nsec);

  for (std::size_t i = 0; i < accelerations.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.z = std::sin(yaws.at(i) * 0.5);
    point.pose.orientation.w = std::cos(yaws.at(i) * 0.5);
    point.acceleration_mps2 = accelerations.at(i);
    point.time_from_start.nanosec =
      sample_times_nsec.empty() ? static_cast<uint32_t>(i * 100000000UL) : sample_times_nsec.at(i);
    trajectory.points.push_back(point);
  }

  return trajectory;
}

}  // namespace

TEST(ExtendedComfortTest, ReturnsAvailableOneForMatchingConsecutiveTrajectories)
{
  const auto previous_trajectory =
    make_trajectory({1.0, 1.2, 1.4, 1.6}, {0.0, 0.01, 0.02, 0.03}, 0);
  const auto current_trajectory =
    make_trajectory({1.2, 1.4, 1.6, 1.8}, {0.01, 0.02, 0.03, 0.04}, 100000000);

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    previous_trajectory, current_trajectory, {});

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.reason, "available");
  EXPECT_DOUBLE_EQ(result.score, 1.0);
}

TEST(ExtendedComfortTest, ReturnsAvailableZeroWhenRmsDiscrepancyExceedsThreshold)
{
  autoware::planning_data_analyzer::metrics::ExtendedComfortParameters parameters;
  parameters.max_acceleration_rms = 0.1;

  const auto previous_trajectory = make_trajectory({1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0}, 0);
  const auto current_trajectory =
    make_trajectory({1.0, 2.0, 3.0, 4.0}, {0.0, 0.0, 0.0, 0.0}, 100000000);

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    previous_trajectory, current_trajectory, parameters);

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.reason, "available");
  EXPECT_DOUBLE_EQ(result.score, 0.0);
}

TEST(ExtendedComfortTest, ReturnsUnavailableForShortTrajectory)
{
  const auto short_trajectory = make_trajectory({1.0, 1.1}, {0.0, 0.01}, 0);
  const auto long_trajectory = make_trajectory({1.0, 1.1, 1.2}, {0.0, 0.01, 0.02}, 100000000);

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    short_trajectory, long_trajectory, {});

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_short_trajectory");
  EXPECT_DOUBLE_EQ(result.score, 0.0);
}

TEST(ExtendedComfortTest, HandlesNonUniformSampleTimes)
{
  const std::vector<uint32_t> times{0U, 100000000U, 250000000U, 450000000U, 700000000U};
  const auto previous_trajectory =
    make_trajectory({1.0, 1.2, 1.4, 1.6, 1.8}, {0.0, 0.01, 0.02, 0.03, 0.04}, 0, times);
  const auto current_trajectory =
    make_trajectory({1.2, 1.4, 1.6, 1.8, 2.0}, {0.01, 0.02, 0.03, 0.04, 0.05}, 100000000, times);

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    previous_trajectory, current_trajectory, {});

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.reason, "available");
}

TEST(ExtendedComfortTest, ReturnsUnavailableForNegativeTimeAlignment)
{
  const auto previous_trajectory =
    make_trajectory({1.0, 1.2, 1.4, 1.6}, {0.0, 0.01, 0.02, 0.03}, 200000000);
  const auto current_trajectory =
    make_trajectory({1.2, 1.4, 1.6, 1.8}, {0.01, 0.02, 0.03, 0.04}, 100000000);

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    previous_trajectory, current_trajectory, {});

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_invalid_time_alignment");
}
