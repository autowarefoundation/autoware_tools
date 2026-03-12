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

#include <sstream>

#define private public
#include "../src/open_loop_evaluator.hpp"
#undef private

#include <gtest/gtest.h>

#include <algorithm>
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

TEST_F(OpenLoopGTSourceModeTest, FormatsDLRHorizonKeysAsExpected)
{
  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  EXPECT_EQ(evaluator.format_horizon_key(0.0), "0s");
  EXPECT_EQ(evaluator.format_horizon_key(0.058), "0.058s");
  EXPECT_EQ(evaluator.format_horizon_key(0.1), "0.1s");
  EXPECT_EQ(evaluator.format_horizon_key(1.23), "1.23s");
  EXPECT_EQ(evaluator.format_horizon_key(2.9999), "3s");
}

TEST_F(OpenLoopGTSourceModeTest, VariantsNamespaceOpenLoopResultTopics)
{
  OpenLoopEvaluator evaluator(
    rclcpp::get_logger("open_loop_gt_source_test"), nullptr,
    OpenLoopEvaluator::GTSourceMode::GT_TRAJECTORY, 200.0);

  evaluator.set_metric_variant("raw");

  EXPECT_EQ(evaluator.metric_topic("ade"), "/open_loop/metrics/raw/ade");
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
  EXPECT_TRUE(has_topic("/trajectory/raw/lateral_accelerations"));
  EXPECT_TRUE(has_topic("/evaluation/compared_trajectory/raw"));
  EXPECT_TRUE(has_topic("/driving_log_replayer/time_step_based_trajectory/raw/results"));
}
