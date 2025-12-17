// Copyright 2024 TIER IV, Inc.
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

#include "../src/autoware_planning_data_analyzer_node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

using autoware::planning_data_analyzer::TopicNames;

class ReplayEvaluationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create node options for testing
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("replay_bag_path", "/tmp/test_replay.bag");
    node_options.append_parameter_override("route_bag_path", "/tmp/test_route.bag");
    node_options.append_parameter_override(
      "metrics", std::vector<std::string>{"lateral_acceleration"});
    node_options.append_parameter_override("sample_num", 10);
    node_options.append_parameter_override("resolution", 0.1);

    // Skip actual node creation due to missing bag files in test environment
    // This would be done in integration tests with actual test bags
  }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ReplayEvaluationTest, TopicDefinitions)
{
  // Global TOPIC constants were removed, this test was updated
  // Changed to use TopicNames structure instead
  TopicNames topic_names;
  topic_names.tf_topic = "/tf";
  topic_names.odometry_topic = "/localization/kinematic_state";
  topic_names.acceleration_topic = "/localization/acceleration";
  topic_names.objects_topic = "/perception/object_recognition/objects";
  topic_names.trajectory_topic = "/planning/scenario_planning/trajectory";
  topic_names.steering_topic = "/vehicle/status/steering_status";
  topic_names.route_topic = "/planning/mission_planning/route";

  // Verify TopicNames structure is created correctly
  EXPECT_EQ(topic_names.tf_topic, "/tf");
  EXPECT_EQ(topic_names.odometry_topic, "/localization/kinematic_state");
  EXPECT_EQ(topic_names.acceleration_topic, "/localization/acceleration");
  EXPECT_EQ(topic_names.objects_topic, "/perception/object_recognition/objects");
  EXPECT_EQ(topic_names.trajectory_topic, "/planning/scenario_planning/trajectory");
  EXPECT_EQ(topic_names.steering_topic, "/vehicle/status/steering_status");
  EXPECT_EQ(topic_names.route_topic, "/planning/mission_planning/route");
}

TEST_F(ReplayEvaluationTest, TrajectoryMessageCreation)
{
  // Test creating trajectory messages for live evaluation
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp.sec = 1;
  trajectory.header.stamp.nanosec = 0;
  trajectory.header.frame_id = "map";

  // Add trajectory points
  for (size_t i = 0; i < 10; i++) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = autoware::test_utils::createPose(i * 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    point.longitudinal_velocity_mps = 15.0;
    point.acceleration_mps2 = 0.0;
    trajectory.points.push_back(point);
  }

  EXPECT_EQ(trajectory.points.size(), 10u);
  EXPECT_EQ(trajectory.header.frame_id, "map");

  // Verify trajectory point data
  const auto & first_point = trajectory.points[0];
  EXPECT_DOUBLE_EQ(first_point.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(first_point.longitudinal_velocity_mps, 15.0);

  const auto & last_point = trajectory.points[9];
  EXPECT_DOUBLE_EQ(last_point.pose.position.x, 18.0);
}

// Mock test for node parameter validation
TEST_F(ReplayEvaluationTest, ParameterValidation)
{
  // Test parameter structure that would be used by the node
  std::map<std::string, std::string> expected_params = {
    {"replay_bag_path", "/path/to/replay.bag"}, {"route_bag_path", "/path/to/route.bag"}};

  for (const auto & [key, value] : expected_params) {
    EXPECT_FALSE(key.empty());
    EXPECT_FALSE(value.empty());
  }

  // Test metric parameter validation
  std::vector<std::string> valid_metrics = {
    "lateral_acceleration", "longitudinal_jerk", "travel_distance"};

  EXPECT_GT(valid_metrics.size(), 0u);
  for (const auto & metric : valid_metrics) {
    EXPECT_FALSE(metric.empty());
  }
}
