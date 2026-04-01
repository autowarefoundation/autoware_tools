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

#include <any>
#include <sstream>

#define private public
#include "../src/open_loop_evaluator.hpp"
#undef private

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using autoware::planning_data_analyzer::OpenLoopEvaluator;
using autoware::planning_data_analyzer::SynchronizedData;
using autoware::planning_data_analyzer::Trajectory;

namespace
{

Trajectory make_trajectory(const rclcpp::Time & start_time, const std::vector<double> & xs)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = static_cast<int32_t>(start_time.seconds());
  trajectory.header.stamp.nanosec =
    static_cast<uint32_t>(start_time.nanoseconds() - trajectory.header.stamp.sec * 1000000000LL);

  for (size_t i = 0; i < xs.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = xs[i];
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    const double time_from_start_s = 0.1 * static_cast<double>(i);
    const int32_t sec = static_cast<int32_t>(time_from_start_s);
    const uint32_t nanosec = static_cast<uint32_t>((time_from_start_s - sec) * 1e9);
    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = nanosec;
    point.longitudinal_velocity_mps = 5.0;
    trajectory.points.push_back(point);
  }

  return trajectory;
}

std::shared_ptr<SynchronizedData> make_sync_data(
  const Trajectory & prediction, const std::shared_ptr<Trajectory> & gt = nullptr)
{
  auto sync_data = std::make_shared<SynchronizedData>();
  sync_data->timestamp = rclcpp::Time(prediction.header.stamp, RCL_ROS_TIME);
  sync_data->bag_timestamp = rclcpp::Time(prediction.header.stamp, RCL_ROS_TIME);
  sync_data->trajectory = std::make_shared<Trajectory>(prediction);
  sync_data->ground_truth_trajectory_msg = gt;
  return sync_data;
}

}  // namespace

class OpenLoopGTSourceModeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(OpenLoopGTSourceModeTest, GTTrajectoryModeSucceedsWithValidGTTopic)
{
  const rclcpp::Time start_time(10, 0);
  const auto prediction = make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.1, 1.1, 2.1, 3.1}));
  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{make_sync_data(prediction, gt)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  EXPECT_NO_THROW(evaluator.evaluate(sync_data_list, nullptr));
  const auto metrics = evaluator.get_metrics();
  ASSERT_EQ(metrics.size(), 1u);
  ASSERT_EQ(metrics.front().num_points, 4u);
  EXPECT_GT(metrics.front().displacement_errors.front(), 0.0);
}

TEST_F(OpenLoopGTSourceModeTest, GTTrajectoryModeSkipsWhenGTIsMissing)
{
  const rclcpp::Time start_time(20, 0);
  const auto prediction = make_trajectory(start_time, {0.0, 1.0, 2.0});
  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{make_sync_data(prediction)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  EXPECT_NO_THROW(evaluator.evaluate(sync_data_list, nullptr));
  EXPECT_TRUE(evaluator.get_metrics().empty());
}

TEST_F(OpenLoopGTSourceModeTest, GTTrajectoryModeUsesSyncToleranceForBoundaryInterpolation)
{
  const rclcpp::Time start_time(30, 0);

  // Predicted trajectory extends to 0.3s while GT extends to 0.2s.
  const auto prediction = make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 1.0, 2.0}));
  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{make_sync_data(prediction, gt)};

  OpenLoopEvaluator strict_evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 50.0);
  EXPECT_NO_THROW(strict_evaluator.evaluate(sync_data_list, nullptr));
  EXPECT_TRUE(strict_evaluator.get_metrics().empty());

  OpenLoopEvaluator tolerant_evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 120.0);
  EXPECT_NO_THROW(tolerant_evaluator.evaluate(sync_data_list, nullptr));
  EXPECT_EQ(tolerant_evaluator.get_metrics().size(), 1u);
}

TEST_F(OpenLoopGTSourceModeTest, VariantsNamespaceOpenLoopResultTopics)
{
  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  evaluator.set_metric_variant("raw");

  EXPECT_EQ(evaluator.metric_topic("ade"), "/open_loop/metrics/raw/ade");
  EXPECT_EQ(evaluator.metric_topic("ahe"), "/open_loop/metrics/raw/ahe");
  EXPECT_EQ(
    evaluator.trajectory_metric_topic("lateral_accelerations"),
    "/trajectory/raw/lateral_accelerations");
  EXPECT_EQ(evaluator.compared_trajectory_topic(), "/evaluation/compared_trajectory/raw");
  EXPECT_EQ(
    evaluator.dlr_result_topic(), "/driving_log_replayer/time_step_based_trajectory/raw/results");

  const auto topics = evaluator.get_result_topics();
  const auto has_topic = [&topics](const std::string & topic_name) {
    return std::any_of(topics.begin(), topics.end(), [&topic_name](const auto & topic) {
      return topic.first == topic_name;
    });
  };

  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/ade"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/ahe"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/fhe"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/history_comfort"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/lane_keeping"));
  EXPECT_TRUE(has_topic("/trajectory/raw/longitudinal_accelerations"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/drivable_area_compliance"));
  EXPECT_TRUE(has_topic("/trajectory/raw/lateral_accelerations"));
  EXPECT_TRUE(has_topic("/trajectory/raw/lateral_jerks"));
  EXPECT_TRUE(has_topic("/trajectory/raw/jerk_magnitudes"));
  EXPECT_TRUE(has_topic("/trajectory/raw/yaw_rates"));
  EXPECT_TRUE(has_topic("/trajectory/raw/yaw_accelerations"));
  EXPECT_TRUE(has_topic("/evaluation/compared_trajectory/raw"));
  EXPECT_TRUE(has_topic("/driving_log_replayer/time_step_based_trajectory/raw/results"));
}

TEST_F(OpenLoopGTSourceModeTest, HistoryComfortIsReportedForComfortableAndUncomfortableTrajectories)
{
  const rclcpp::Time start_time(50, 0);

  auto comfortable_prediction = make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5});
  auto uncomfortable_prediction = make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5}));

  for (size_t i = 0; i < comfortable_prediction.points.size(); ++i) {
    comfortable_prediction.points[i].longitudinal_velocity_mps = 2.0;
    uncomfortable_prediction.points[i].longitudinal_velocity_mps = 2.0;
  }

  uncomfortable_prediction.points[1].longitudinal_velocity_mps = 3.0;
  uncomfortable_prediction.points[2].longitudinal_velocity_mps = 5.0;
  uncomfortable_prediction.points[3].longitudinal_velocity_mps = 6.0;

  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{
    make_sync_data(comfortable_prediction, gt), make_sync_data(uncomfortable_prediction, gt)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  evaluator.evaluate(sync_data_list, nullptr);

  const auto metrics = evaluator.get_metrics();
  ASSERT_EQ(metrics.size(), 2u);
  EXPECT_DOUBLE_EQ(metrics[0].history_comfort, 1.0);
  EXPECT_DOUBLE_EQ(metrics[1].history_comfort, 0.0);

  const auto full_json = evaluator.get_full_results_as_json();
  ASSERT_EQ(full_json["trajectories"].size(), 2u);
  EXPECT_DOUBLE_EQ(full_json["trajectories"][0]["history_comfort"].get<double>(), 1.0);
  EXPECT_DOUBLE_EQ(full_json["trajectories"][1]["history_comfort"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(
    full_json["trajectories"][0]["trajectory_point_metrics"]["history_comfort"].get<double>(), 1.0);
  EXPECT_DOUBLE_EQ(
    full_json["trajectories"][1]["trajectory_point_metrics"]["history_comfort"].get<double>(), 0.0);

  const auto summary_json = evaluator.get_summary_as_json();
  EXPECT_DOUBLE_EQ(summary_json["aggregate/history_comfort/mean"].get<double>(), 0.5);
  EXPECT_DOUBLE_EQ(summary_json["aggregate/history_comfort/min"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(summary_json["aggregate/history_comfort/max"].get<double>(), 1.0);
  EXPECT_EQ(summary_json["aggregate/lane_keeping_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(summary_json["aggregate/lane_keeping_unavailable_count"].get<std::size_t>(), 2u);
}

TEST_F(OpenLoopGTSourceModeTest, DACUnavailableReasonIsReportedWhenRouteHandlerIsMissing)
{
  const rclcpp::Time start_time(60, 0);
  const auto prediction = make_trajectory(start_time, {0.0, 1.0, 2.0});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 1.0, 2.0}));
  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{make_sync_data(prediction, gt)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  evaluator.evaluate(sync_data_list, nullptr);

  const auto full_json = evaluator.get_full_results_as_json();
  ASSERT_EQ(full_json["trajectories"].size(), 1u);
  EXPECT_FALSE(full_json["trajectories"][0]["lane_keeping_available"].get<bool>());
  EXPECT_EQ(
    full_json["trajectories"][0]["lane_keeping_reason"].get<std::string>(),
    "unavailable_no_route_handler");
  EXPECT_FALSE(full_json["trajectories"][0]["drivable_area_compliance_available"].get<bool>());
  EXPECT_EQ(
    full_json["trajectories"][0]["drivable_area_compliance_reason"].get<std::string>(),
    "unavailable_no_route_handler");

  const auto summary_json = evaluator.get_summary_as_json();
  ASSERT_TRUE(summary_json.contains("aggregate/lane_keeping_reason_counts"));
  EXPECT_EQ(
    summary_json["aggregate/lane_keeping_reason_counts"]["unavailable_no_route_handler"]
      .get<std::size_t>(),
    1u);
  ASSERT_TRUE(summary_json.contains("aggregate/drivable_area_compliance_reason_counts"));
  EXPECT_EQ(
    summary_json["aggregate/drivable_area_compliance_reason_counts"]["unavailable_no_route_handler"]
      .get<std::size_t>(),
    1u);
}

TEST_F(
  OpenLoopGTSourceModeTest,
  HistoryComfortUsesYawRateForLateralAccelerationWhenLateralVelocityIsZero)
{
  const rclcpp::Time start_time(55, 0);

  auto turning_prediction = make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0}));

  for (size_t i = 0; i < turning_prediction.points.size(); ++i) {
    turning_prediction.points[i].longitudinal_velocity_mps = 10.0;
    turning_prediction.points[i].lateral_velocity_mps = 0.0;
  }

  for (size_t i = 0; i < turning_prediction.points.size(); ++i) {
    const double yaw = 0.05 * static_cast<double>(i);
    turning_prediction.points[i].pose.orientation.z = std::sin(yaw * 0.5);
    turning_prediction.points[i].pose.orientation.w = std::cos(yaw * 0.5);
  }

  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{
    make_sync_data(turning_prediction, gt)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  evaluator.evaluate(sync_data_list, nullptr);

  const auto metrics = evaluator.get_metrics();
  ASSERT_EQ(metrics.size(), 1u);
  EXPECT_DOUBLE_EQ(metrics[0].history_comfort, 0.0);

  const auto full_json = evaluator.get_full_results_as_json();
  ASSERT_EQ(full_json["trajectories"].size(), 1u);
  EXPECT_DOUBLE_EQ(full_json["trajectories"][0]["history_comfort"].get<double>(), 0.0);
}

TEST_F(OpenLoopGTSourceModeTest, HeadingMetricsUseWrappedYawErrorPerHorizon)
{
  const rclcpp::Time start_time(40, 0);
  auto prediction = make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0}));

  prediction.points[0].pose.orientation.z = std::sin((-M_PI + 0.1) * 0.5);
  prediction.points[0].pose.orientation.w = std::cos((-M_PI + 0.1) * 0.5);
  gt->points[0].pose.orientation.z = std::sin((M_PI - 0.1) * 0.5);
  gt->points[0].pose.orientation.w = std::cos((M_PI - 0.1) * 0.5);

  prediction.points[1].pose.orientation.z = std::sin(0.2 * 0.5);
  prediction.points[1].pose.orientation.w = std::cos(0.2 * 0.5);
  gt->points[1].pose.orientation.w = 1.0;

  prediction.points[2].pose.orientation.z = std::sin((-0.3) * 0.5);
  prediction.points[2].pose.orientation.w = std::cos((-0.3) * 0.5);
  gt->points[2].pose.orientation.w = 1.0;

  prediction.points[3].pose.orientation.z = std::sin(0.4 * 0.5);
  prediction.points[3].pose.orientation.w = std::cos(0.4 * 0.5);
  gt->points[3].pose.orientation.w = 1.0;

  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{make_sync_data(prediction, gt)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);
  {
    // Avoid initializer list to prevent GCC 13 false-positive -Wstringop-overread
    std::vector<double> horizons;
    horizons.push_back(0.2);
    evaluator.set_evaluation_horizons(horizons);
  }

  evaluator.evaluate(sync_data_list, nullptr);
  const auto metrics = evaluator.get_metrics();
  ASSERT_EQ(metrics.size(), 1u);
  ASSERT_EQ(metrics.front().ahe.size(), 4u);
  ASSERT_EQ(metrics.front().heading_errors.size(), 4u);

  EXPECT_NEAR(metrics.front().ahe[0], 0.2, 1e-6);
  EXPECT_NEAR(metrics.front().ahe[1], 0.2, 1e-6);
  EXPECT_NEAR(metrics.front().ahe[2], 0.233333333333, 1e-6);
  EXPECT_NEAR(metrics.front().ahe[3], 0.275, 1e-6);
  EXPECT_NEAR(metrics.front().heading_errors[0], 0.2, 1e-6);
  EXPECT_NEAR(metrics.front().heading_errors[1], 0.2, 1e-6);
  EXPECT_NEAR(metrics.front().heading_errors[2], 0.3, 1e-6);
  EXPECT_NEAR(metrics.front().heading_errors[3], 0.4, 1e-6);

  ASSERT_EQ(metrics.front().horizon_results.size(), 2u);
  EXPECT_EQ(metrics.front().horizon_results[0].first, "full");
  EXPECT_NEAR(metrics.front().horizon_results[0].second.ahe, 0.275, 1e-6);
  EXPECT_NEAR(metrics.front().horizon_results[0].second.fhe, 0.4, 1e-6);

  EXPECT_EQ(metrics.front().horizon_results[1].first, "0.2s");
  EXPECT_NEAR(metrics.front().horizon_results[1].second.ahe, 0.233333333333, 1e-6);
  EXPECT_NEAR(metrics.front().horizon_results[1].second.fhe, 0.3, 1e-6);

  const auto summary_json = evaluator.get_summary_as_json();
  EXPECT_NEAR(summary_json["full/ahe/mean"].get<double>(), 0.275, 1e-6);
  EXPECT_NEAR(summary_json["full/fhe/mean"].get<double>(), 0.4, 1e-6);
  EXPECT_NEAR(summary_json["0.2s/ahe/mean"].get<double>(), 0.233333333333, 1e-6);
  EXPECT_NEAR(summary_json["0.2s/fhe/mean"].get<double>(), 0.3, 1e-6);
  EXPECT_DOUBLE_EQ(summary_json["aggregate/drivable_area_compliance/mean"].get<double>(), 0.0);
  EXPECT_EQ(
    summary_json["aggregate/drivable_area_compliance_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(
    summary_json["aggregate/drivable_area_compliance_unavailable_count"].get<std::size_t>(), 1u);
}
