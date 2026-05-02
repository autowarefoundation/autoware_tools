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

#include "../../src/metrics/epdms/subscores/ego_progress.hpp"

#include <gtest/gtest.h>

#include <memory>

TEST(EgoProgressTest, ReturnsUnavailableWhenSelectedTrajectoryIsMissing)
{
  const auto result =
    autoware::planning_data_analyzer::metrics::calculate_ego_progress(nullptr, nullptr, nullptr);

  EXPECT_FALSE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "unavailable_no_selected_trajectory");
}

TEST(EgoProgressTest, ReturnsUnavailableWhenRouteHandlerIsMissing)
{
  auto trajectory = std::make_shared<autoware::planning_data_analyzer::Trajectory>();
  trajectory->header.frame_id = "map";
  trajectory->points.resize(2);

  const auto result =
    autoware::planning_data_analyzer::metrics::calculate_ego_progress(trajectory, nullptr, nullptr);

  EXPECT_FALSE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "unavailable_no_route_handler");
}
