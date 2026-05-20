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

#include "metrics/epdms/subscores/lane_keeping.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <utility>
#include <vector>

using autoware::planning_data_analyzer::metrics::LaneKeepingEvaluationPoint;
using autoware::planning_data_analyzer::metrics::LaneKeepingParameters;

namespace
{

LaneKeepingEvaluationPoint make_evaluation_point(
  const double seconds, const double lateral_deviation, const bool is_in_intersection = false,
  const double speed_mps = 5.0, const double cumulative_progress_m = 0.0)
{
  return LaneKeepingEvaluationPoint{
    rclcpp::Duration::from_seconds(seconds), lateral_deviation, is_in_intersection, speed_mps,
    cumulative_progress_m};
}

LaneKeepingParameters make_parameters()
{
  LaneKeepingParameters parameters;
  parameters.max_lateral_deviation = 0.5;
  parameters.max_continuous_violation_time = 2.0;
  return parameters;
}

}  // namespace

TEST(LaneKeepingTest, ReturnsOneWhenAllDeviationsStayWithinThreshold)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.1), make_evaluation_point(1.0, -0.3),
    make_evaluation_point(2.0, 0.5)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters());

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, ReturnsOneForShortThresholdSpike)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.0), make_evaluation_point(0.5, 0.8),
    make_evaluation_point(1.0, 0.1)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters());

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, ReturnsZeroForContinuousViolationLongerThanWindow)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.7), make_evaluation_point(1.0, 0.8),
    make_evaluation_point(2.1, 0.9)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters());

  EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(LaneKeepingTest, IgnoresViolationsInsideIntersections)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.8, true), make_evaluation_point(1.0, 0.9, true),
    make_evaluation_point(2.5, 0.1, false)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters());

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, ReturnsZeroWhenSamplesAreEmpty)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points;

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters());

  EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(LaneKeepingTest, NonFiniteGapBreaksContinuousViolationWindow)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.7),
    make_evaluation_point(1.0, std::numeric_limits<double>::quiet_NaN()),
    make_evaluation_point(2.1, 0.9)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters());

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, LaneChangeWindowBreaksViolationRun)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.7), make_evaluation_point(1.0, 0.8),
    make_evaluation_point(2.1, 0.9)};
  const std::vector<std::pair<double, double>> lane_change_windows{{0.5, 1.5}};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, make_parameters(), lane_change_windows);

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, QueueExemptionAndReleaseGraceBreakViolationRun)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.7, false, 0.0, 0.0),
    make_evaluation_point(1.0, 0.8, false, 0.0, 0.1),
    make_evaluation_point(2.1, 0.9, false, 5.0, 0.2),
    make_evaluation_point(3.6, 0.9, false, 5.0, 6.0)};
  auto parameters = make_parameters();
  parameters.queue_release_grace_time = 1.5;

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, parameters);

  EXPECT_DOUBLE_EQ(score, 1.0);
}
