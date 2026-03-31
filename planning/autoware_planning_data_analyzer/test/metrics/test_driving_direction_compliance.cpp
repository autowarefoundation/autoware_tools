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

#include "../../src/metrics/driving_direction_compliance.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::planning_data_analyzer::metrics
{
namespace
{

DrivingDirectionEvaluationPoint make_point(
  const double t_s, const double progress_m, const bool oncoming, const bool intersection = false)
{
  builtin_interfaces::msg::Duration duration{};
  duration.sec = static_cast<int32_t>(t_s);
  duration.nanosec = static_cast<uint32_t>((t_s - static_cast<double>(duration.sec)) * 1e9);
  return DrivingDirectionEvaluationPoint{
    duration, progress_m, oncoming, intersection};
}

}  // namespace

TEST(DrivingDirectionCompliance, EmptyPointsUnavailable)
{
  const auto result = calculate_driving_direction_compliance({});
  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_no_evaluation_points");
}

TEST(DrivingDirectionCompliance, SmallOncomingProgressPasses)
{
  const std::vector<DrivingDirectionEvaluationPoint> points{
    make_point(0.0, 0.0, false), make_point(0.1, 0.4, true), make_point(0.2, 0.5, true),
    make_point(0.3, 0.6, true)};
  const auto result = calculate_driving_direction_compliance(points);
  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(DrivingDirectionCompliance, MediumOncomingProgressGetsHalfPenalty)
{
  const std::vector<DrivingDirectionEvaluationPoint> points{
    make_point(0.0, 0.0, false), make_point(0.1, 1.0, true), make_point(0.2, 1.2, true),
    make_point(0.3, 0.8, true)};
  const auto result = calculate_driving_direction_compliance(points);
  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.5);
  EXPECT_EQ(result.reason, "minor_oncoming_progress");
}

TEST(DrivingDirectionCompliance, LargeOncomingProgressFails)
{
  const std::vector<DrivingDirectionEvaluationPoint> points{
    make_point(0.0, 0.0, false), make_point(0.1, 2.5, true), make_point(0.2, 2.4, true),
    make_point(0.3, 1.5, true)};
  const auto result = calculate_driving_direction_compliance(points);
  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "major_oncoming_progress");
}

TEST(DrivingDirectionCompliance, IntersectionSamplesAreIgnored)
{
  const std::vector<DrivingDirectionEvaluationPoint> points{
    make_point(0.0, 0.0, false), make_point(0.1, 3.0, true, true), make_point(0.2, 2.0, true, true)};
  const auto result = calculate_driving_direction_compliance(points);
  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.max_oncoming_progress_m, 0.0);
}

}  // namespace autoware::planning_data_analyzer::metrics
