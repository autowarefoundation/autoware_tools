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
  const std::vector<double> & velocities, const std::vector<double> & yaws)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "map";

  for (std::size_t i = 0; i < velocities.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.z = std::sin(yaws.at(i) * 0.5);
    point.pose.orientation.w = std::cos(yaws.at(i) * 0.5);
    point.longitudinal_velocity_mps = velocities.at(i);
    point.time_from_start.nanosec = static_cast<uint32_t>(i * 100000000UL);
    trajectory.points.push_back(point);
  }

  return trajectory;
}

}  // namespace

TEST(ExtendedComfortTest, ReturnsAvailableOneForMatchingConsecutiveTrajectories)
{
  const auto trajectory = make_trajectory({1.0, 1.2, 1.4, 1.6}, {0.0, 0.01, 0.02, 0.03});

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    trajectory, trajectory, {});

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.reason, "available");
  EXPECT_DOUBLE_EQ(result.score, 1.0);
}

TEST(ExtendedComfortTest, ReturnsAvailableZeroWhenRmsDiscrepancyExceedsThreshold)
{
  autoware::planning_data_analyzer::metrics::ExtendedComfortParameters parameters;
  parameters.max_acceleration_rms = 0.1;

  const auto previous_trajectory = make_trajectory({1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0});
  const auto current_trajectory = make_trajectory({1.0, 2.0, 3.0, 4.0}, {0.0, 0.0, 0.0, 0.0});

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    previous_trajectory, current_trajectory, parameters);

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.reason, "available");
  EXPECT_DOUBLE_EQ(result.score, 0.0);
}

TEST(ExtendedComfortTest, ReturnsUnavailableForShortTrajectory)
{
  const auto short_trajectory = make_trajectory({1.0, 1.1}, {0.0, 0.01});
  const auto long_trajectory = make_trajectory({1.0, 1.1, 1.2}, {0.0, 0.01, 0.02});

  const auto result = autoware::planning_data_analyzer::metrics::calculate_extended_comfort(
    short_trajectory, long_trajectory, {});

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_short_trajectory");
  EXPECT_DOUBLE_EQ(result.score, 0.0);
}
