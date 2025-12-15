// Copyright 2025 TIER IV, Inc.
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

#include "../../src/metrics/deviation_metrics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(DeviationMetrics, Distance2D)
{
  geometry_msgs::msg::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = 0.0;

  geometry_msgs::msg::Point p2;
  p2.x = 3.0;
  p2.y = 4.0;
  p2.z = 0.0;

  double dist = metrics::calculate_distance_2d(p1, p2);

  EXPECT_DOUBLE_EQ(dist, 5.0);  // 3-4-5 triangle
}

TEST(DeviationMetrics, DisplacementErrors_PerfectMatch)
{
  std::vector<geometry_msgs::msg::Point> predicted(3);
  std::vector<geometry_msgs::msg::Point> ground_truth(3);

  for (size_t i = 0; i < 3; ++i) {
    predicted[i].x = i * 1.0;
    predicted[i].y = 0.0;
    ground_truth[i] = predicted[i];  // Perfect match
  }

  auto result = metrics::calculate_displacement_errors(predicted, ground_truth);

  EXPECT_DOUBLE_EQ(result.ade, 0.0);
  EXPECT_DOUBLE_EQ(result.fde, 0.0);
  ASSERT_EQ(result.point_wise_errors.size(), 3u);
  for (const auto & err : result.point_wise_errors) {
    EXPECT_DOUBLE_EQ(err, 0.0);
  }
}

TEST(DeviationMetrics, DisplacementErrors_ConstantOffset)
{
  std::vector<geometry_msgs::msg::Point> predicted(3);
  std::vector<geometry_msgs::msg::Point> ground_truth(3);

  for (size_t i = 0; i < 3; ++i) {
    predicted[i].x = i * 1.0;
    predicted[i].y = 0.0;

    ground_truth[i].x = i * 1.0;
    ground_truth[i].y = 1.0;  // 1m lateral offset
  }

  auto result = metrics::calculate_displacement_errors(predicted, ground_truth);

  EXPECT_DOUBLE_EQ(result.ade, 1.0);
  EXPECT_DOUBLE_EQ(result.fde, 1.0);
}

TEST(DeviationMetrics, DisplacementErrors_IncreasingError)
{
  std::vector<geometry_msgs::msg::Point> predicted(3);
  std::vector<geometry_msgs::msg::Point> ground_truth(3);

  predicted[0].x = 0.0;
  predicted[0].y = 0.0;
  predicted[1].x = 1.0;
  predicted[1].y = 0.0;
  predicted[2].x = 2.0;
  predicted[2].y = 0.0;

  ground_truth[0].x = 0.0;
  ground_truth[0].y = 0.0;  // error = 0
  ground_truth[1].x = 1.0;
  ground_truth[1].y = 0.5;  // error = 0.5
  ground_truth[2].x = 2.0;
  ground_truth[2].y = 1.0;  // error = 1.0

  auto result = metrics::calculate_displacement_errors(predicted, ground_truth);

  EXPECT_NEAR(result.ade, 0.5, 0.01);  // (0 + 0.5 + 1.0) / 3 = 0.5
  EXPECT_DOUBLE_EQ(result.fde, 1.0);
}

TEST(DeviationMetrics, ErrorsInVehicleFrame_Longitudinal)
{
  geometry_msgs::msg::Pose traj_pose;
  traj_pose.position.x = 1.0;
  traj_pose.position.y = 0.0;
  traj_pose.orientation.w = 1.0;  // 0 rad

  geometry_msgs::msg::Pose gt_pose;
  gt_pose.position.x = 0.0;
  gt_pose.position.y = 0.0;
  gt_pose.orientation.w = 1.0;

  auto [lon_err, lat_err] = metrics::calculate_errors_in_vehicle_frame(traj_pose, gt_pose);

  EXPECT_NEAR(lon_err, 1.0, 0.01);  // 1m forward
  EXPECT_NEAR(lat_err, 0.0, 0.01);  // No lateral deviation
}

TEST(DeviationMetrics, ErrorsInVehicleFrame_Lateral)
{
  geometry_msgs::msg::Pose traj_pose;
  traj_pose.position.x = 0.0;
  traj_pose.position.y = 1.0;
  traj_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose gt_pose;
  gt_pose.position.x = 0.0;
  gt_pose.position.y = 0.0;
  gt_pose.orientation.w = 1.0;

  auto [lon_err, lat_err] = metrics::calculate_errors_in_vehicle_frame(traj_pose, gt_pose);

  EXPECT_NEAR(lon_err, 0.0, 0.01);  // No longitudinal deviation
  EXPECT_NEAR(lat_err, 1.0, 0.01);  // 1m to the left
}

TEST(DeviationMetrics, FrenetErrors_MultiplePoints)
{
  std::vector<geometry_msgs::msg::Pose> predicted(5);
  std::vector<geometry_msgs::msg::Pose> ground_truth(5);

  // Predicted: straight line at y = 0.5
  // Ground truth: straight line at y = 0
  for (size_t i = 0; i < 5; ++i) {
    predicted[i].position.x = i * 1.0;
    predicted[i].position.y = 0.5;
    predicted[i].orientation.w = 1.0;

    ground_truth[i].position.x = i * 1.0;
    ground_truth[i].position.y = 0.0;
    ground_truth[i].orientation.w = 1.0;
  }

  auto result = metrics::calculate_frenet_errors(predicted, ground_truth);

  EXPECT_EQ(result.lateral_errors.size(), 5u);
  EXPECT_EQ(result.longitudinal_errors.size(), 5u);

  // All points have 0.5m lateral deviation
  for (const auto & lat_err : result.lateral_errors) {
    EXPECT_NEAR(std::abs(lat_err), 0.5, 0.01);
  }

  // Check statistics
  EXPECT_NEAR(std::abs(result.lateral_stats.mean), 0.5, 0.01);
  EXPECT_NEAR(result.lateral_stats.max_val, 0.5, 0.01);
}

TEST(MetricTypes, StatisticsCalculation_Simple)
{
  std::vector<double> values = {1.0, 2.0, 3.0, 4.0, 5.0};

  auto stats = metrics::calculate_statistics(values);

  EXPECT_DOUBLE_EQ(stats.mean, 3.0);
  EXPECT_DOUBLE_EQ(stats.min_val, 1.0);
  EXPECT_DOUBLE_EQ(stats.max_val, 5.0);
  EXPECT_NEAR(stats.std_dev, std::sqrt(2.0), 0.01);  // variance = 2.0
}

TEST(MetricTypes, StatisticsCalculation_Empty)
{
  std::vector<double> values = {};

  auto stats = metrics::calculate_statistics(values);

  EXPECT_DOUBLE_EQ(stats.mean, 0.0);
  EXPECT_DOUBLE_EQ(stats.min_val, 0.0);
  EXPECT_DOUBLE_EQ(stats.max_val, 0.0);
  EXPECT_DOUBLE_EQ(stats.std_dev, 0.0);
}

TEST(MetricTypes, StatisticsCalculation_SingleValue)
{
  std::vector<double> values = {42.0};

  auto stats = metrics::calculate_statistics(values);

  EXPECT_DOUBLE_EQ(stats.mean, 42.0);
  EXPECT_DOUBLE_EQ(stats.min_val, 42.0);
  EXPECT_DOUBLE_EQ(stats.max_val, 42.0);
  EXPECT_DOUBLE_EQ(stats.std_dev, 0.0);
}
