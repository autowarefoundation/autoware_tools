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

#include "metrics/evaluator/evaluator.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics::evaluator
{
namespace
{

EvaluatorMetricMeasurement make_measurement(
  const double value, const std::vector<std::string> & matched_rules = {})
{
  EvaluatorMetricMeasurement measurement;
  measurement.value = value;
  measurement.matched_exclusion_rules = matched_rules;
  return measurement;
}

std::shared_ptr<Odometry> make_odometry(const double x, const double y)
{
  auto odometry = std::make_shared<Odometry>();
  odometry->pose.pose.position.x = x;
  odometry->pose.pose.position.y = y;
  return odometry;
}

const EvaluatorMetricStats & find_metric_stats(
  const std::vector<EvaluatorMetricResult> & results, const std::string & metric_key,
  const std::string & rule)
{
  for (const auto & result : results) {
    if (result.metric_key == metric_key && result.rule == rule) {
      return result.stats;
    }
  }
  throw std::runtime_error("metric result not found: " + metric_key + ", rule=" + rule);
}

TEST(EvaluatorMetrics, FilterOdometryOutsideInitialPose)
{
  const std::vector<std::shared_ptr<Odometry>> kinematic_states = {
    make_odometry(10.0, 20.0), make_odometry(10.1, 20.0), make_odometry(10.0, 21.5),
    make_odometry(15.0, 20.0)};

  const auto filtered = filter_odometry_outside_initial_pose(kinematic_states, 2.0);
  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_DOUBLE_EQ(filtered.front()->pose.pose.position.x, 15.0);
  EXPECT_DOUBLE_EQ(filtered.front()->pose.pose.position.y, 20.0);
}

TEST(EvaluatorMetrics, InsideOutsideIntersectionArea)
{
  const std::vector<EvaluatorMetricMeasurement> measurements = {
    make_measurement(0.1, {}), make_measurement(2.0, {"intersection_area"}),
    make_measurement(-0.2, {})};

  const auto results = aggregate_metric_measurements(
    measurements, {"intersection_area"}, "lateral_deviation_centerline");

  EXPECT_EQ(find_metric_stats(results, "lateral_deviation_centerline", "all").count, 3U);
  EXPECT_EQ(
    find_metric_stats(results, "lateral_deviation_centerline", "outside_intersection_area").count,
    2U);
  EXPECT_EQ(
    find_metric_stats(results, "lateral_deviation_centerline", "in_intersection_area").count, 1U);
}

TEST(EvaluatorConfig, LoadFromYamlFixture)
{
  const auto path =
    std::filesystem::path(__FILE__).parent_path() / "fixtures" / "evaluator_config.fixture.yaml";
  ASSERT_TRUE(std::filesystem::exists(path)) << path;

  const auto configs = load_evaluator_configs_from_yaml_file(path.string());
  ASSERT_EQ(configs.size(), 1U);
  EXPECT_EQ(configs.front().exclusion_rules.front(), "intersection_area");
}

TEST(EvaluatorMetrics, GroupJsonUsesExpectedPaths)
{
  const auto results = aggregate_metric_measurements(
    {make_measurement(0.1, {}), make_measurement(1.0, {"intersection_area"})},
    {"intersection_area"}, "lateral_deviation_centerline");
  const auto json = evaluator_metric_results_to_json(results);

  EXPECT_TRUE(json.contains("outside_intersection_area/lateral_deviation_centerline/mean"));
  EXPECT_TRUE(
    json.contains("outside_intersection_area/lateral_deviation_centerline/percentile_95"));
  EXPECT_TRUE(json.contains("in_intersection_area/lateral_deviation_centerline/count"));
  EXPECT_FALSE(json.contains("all/description"));
}

}  // namespace
}  // namespace autoware::planning_data_analyzer::metrics::evaluator
