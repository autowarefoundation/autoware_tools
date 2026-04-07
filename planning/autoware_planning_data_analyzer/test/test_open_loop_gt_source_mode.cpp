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

  const auto topics = evaluator.get_result_topics();
  const auto has_topic = [&topics](const std::string & topic_name) {
    return std::any_of(topics.begin(), topics.end(), [&topic_name](const auto & topic) {
      return topic.first == topic_name;
    });
  };

  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/history_comfort"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/extended_comfort"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/time_to_collision_within_bound"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/lane_keeping"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/ego_progress"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/drivable_area_compliance"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/no_at_fault_collision"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/driving_direction_compliance"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/traffic_light_compliance"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/synthetic_epdms_raw"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/synthetic_epdms_raw_available"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/synthetic_epdms_human_filtered"));
  EXPECT_TRUE(has_topic("/open_loop/metrics/raw/synthetic_epdms_human_filtered_available"));
}

TEST_F(OpenLoopGTSourceModeTest, HistoryComfortIsReportedForComfortableAndUncomfortableTrajectories)
{
  const rclcpp::Time start_time(50, 0);
  auto comfortable_prediction = make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5});
  auto uncomfortable_prediction = make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5}));

  for (auto & point : comfortable_prediction.points) {
    point.longitudinal_velocity_mps = 2.0;
  }
  for (auto & point : uncomfortable_prediction.points) {
    point.longitudinal_velocity_mps = 2.0;
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

  const auto summary_json = evaluator.get_summary_as_json();
  EXPECT_DOUBLE_EQ(summary_json["aggregate/history_comfort/mean"].get<double>(), 0.5);
  EXPECT_EQ(summary_json["aggregate/extended_comfort_available_count"].get<std::size_t>(), 1u);
  EXPECT_EQ(summary_json["aggregate/extended_comfort_unavailable_count"].get<std::size_t>(), 1u);
  EXPECT_EQ(summary_json["aggregate/lane_keeping_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(summary_json["aggregate/lane_keeping_unavailable_count"].get<std::size_t>(), 2u);
  EXPECT_EQ(summary_json["aggregate/ego_progress_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(summary_json["aggregate/ego_progress_unavailable_count"].get<std::size_t>(), 2u);
  EXPECT_EQ(summary_json["aggregate/no_at_fault_collision_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(
    summary_json["aggregate/no_at_fault_collision_unavailable_count"].get<std::size_t>(), 2u);
  EXPECT_EQ(
    summary_json["aggregate/driving_direction_compliance_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(
    summary_json["aggregate/driving_direction_compliance_unavailable_count"].get<std::size_t>(),
    2u);
  EXPECT_EQ(
    summary_json["aggregate/traffic_light_compliance_available_count"].get<std::size_t>(), 0u);
  EXPECT_EQ(
    summary_json["aggregate/traffic_light_compliance_unavailable_count"].get<std::size_t>(), 2u);
}

TEST_F(OpenLoopGTSourceModeTest, HumanFilterPromotesAgentHistoryComfortWhenHumanIsAlsoZero)
{
  const rclcpp::Time start_time(55, 0);
  auto prediction = make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5});
  auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5}));

  for (auto & point : prediction.points) {
    point.longitudinal_velocity_mps = 2.0;
  }
  for (auto & point : gt->points) {
    point.longitudinal_velocity_mps = 2.0;
  }
  prediction.points[1].longitudinal_velocity_mps = 3.0;
  prediction.points[2].longitudinal_velocity_mps = 5.0;
  prediction.points[3].longitudinal_velocity_mps = 6.0;
  gt->points[1].longitudinal_velocity_mps = 3.0;
  gt->points[2].longitudinal_velocity_mps = 5.0;
  gt->points[3].longitudinal_velocity_mps = 6.0;

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);
  evaluator.evaluate({make_sync_data(prediction, gt)}, nullptr);

  const auto full_json = evaluator.get_full_results_as_json();
  ASSERT_EQ(full_json["trajectories"].size(), 1u);
  EXPECT_DOUBLE_EQ(full_json["trajectories"][0]["history_comfort"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(
    full_json["trajectories"][0]["human_reference"]["history_comfort"].get<double>(), 0.0);
  EXPECT_TRUE(
    full_json["trajectories"][0]["human_filtered"]["history_comfort_filter_applied"].get<bool>());
  EXPECT_DOUBLE_EQ(
    full_json["trajectories"][0]["human_filtered"]["history_comfort"].get<double>(), 1.0);

  const auto summary_json = evaluator.get_summary_as_json();
  EXPECT_EQ(
    summary_json["aggregate/human_filter_applied_counts/history_comfort"].get<std::size_t>(), 1u);
  EXPECT_DOUBLE_EQ(
    summary_json["aggregate/human_filtered/history_comfort/mean"].get<double>(), 1.0);
}

TEST_F(OpenLoopGTSourceModeTest, MissingInputsUnavailableReasonsAreReported)
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
  EXPECT_EQ(
    full_json["trajectories"][0]["lane_keeping_reason"].get<std::string>(),
    "unavailable_no_route_handler");
  EXPECT_EQ(
    full_json["trajectories"][0]["ego_progress_reason"].get<std::string>(),
    "unavailable_no_route_handler");
  EXPECT_EQ(
    full_json["trajectories"][0]["drivable_area_compliance_reason"].get<std::string>(),
    "unavailable_no_route_handler");
  EXPECT_EQ(
    full_json["trajectories"][0]["driving_direction_compliance_reason"].get<std::string>(),
    "unavailable_no_route_handler");
  EXPECT_EQ(
    full_json["trajectories"][0]["traffic_light_compliance_reason"].get<std::string>(),
    "unavailable_no_route_handler");
  EXPECT_EQ(
    full_json["trajectories"][0]["no_at_fault_collision_reason"].get<std::string>(),
    "unavailable_no_objects_message");
}

TEST_F(OpenLoopGTSourceModeTest, ExtendedComfortAvailabilityIsReportedAcrossConsecutivePlans)
{
  const rclcpp::Time start_time(65, 0);
  auto first_prediction = make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5});
  auto second_prediction =
    make_trajectory(start_time + rclcpp::Duration::from_seconds(0.1), {0.0, 0.5, 1.1, 1.7});
  const auto first_gt =
    std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 0.5, 1.0, 1.5}));
  const auto second_gt = std::make_shared<Trajectory>(
    make_trajectory(start_time + rclcpp::Duration::from_seconds(0.1), {0.0, 0.5, 1.1, 1.7}));

  for (auto * prediction : {&first_prediction, &second_prediction}) {
    double velocity_mps = 2.0;
    for (auto & point : prediction->points) {
      point.longitudinal_velocity_mps = velocity_mps;
      velocity_mps += 0.1;
    }
  }

  std::vector<std::shared_ptr<SynchronizedData>> sync_data_list{
    make_sync_data(first_prediction, first_gt), make_sync_data(second_prediction, second_gt)};

  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  evaluator.evaluate(sync_data_list, nullptr);

  const auto metrics = evaluator.get_metrics();
  ASSERT_EQ(metrics.size(), 2u);
  EXPECT_FALSE(metrics[0].extended_comfort_available);
  EXPECT_TRUE(metrics[1].extended_comfort_available);
}

TEST_F(
  OpenLoopGTSourceModeTest,
  HistoryComfortUsesYawRateForLateralAccelerationWhenLateralVelocityIsZero)
{
  const rclcpp::Time start_time(55, 0);

  auto turning_prediction = make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0});
  const auto gt = std::make_shared<Trajectory>(make_trajectory(start_time, {0.0, 1.0, 2.0, 3.0}));

  for (auto & point : turning_prediction.points) {
    point.longitudinal_velocity_mps = 10.0;
    point.lateral_velocity_mps = 0.0;
  }

  double yaw = 0.0;
  for (auto & point : turning_prediction.points) {
    point.pose.orientation.z = std::sin(yaw * 0.5);
    point.pose.orientation.w = std::cos(yaw * 0.5);
    yaw += 0.05;
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
  evaluator.set_evaluation_horizons({0.2});

  evaluator.evaluate(sync_data_list, nullptr);
  const auto metrics = evaluator.get_metrics();
  ASSERT_EQ(metrics.size(), 1u);
  ASSERT_EQ(metrics.front().ahe.size(), 4u);
  ASSERT_EQ(metrics.front().heading_errors.size(), 4u);

  EXPECT_NEAR(metrics.front().ahe[0], 0.2, 1e-6);
  EXPECT_NEAR(metrics.front().ahe[1], 0.2, 1e-6);
  EXPECT_NEAR(metrics.front().ahe[2], 0.233333333333, 1e-6);
  EXPECT_NEAR(metrics.front().ahe[3], 0.275, 1e-6);
}
