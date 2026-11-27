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

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/lane_goal.hpp"
#include "closed_loop/lane_layout.hpp"
#include "straight_road.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

using autoware_perception_msgs::msg::ObjectClassification;
using test::StraightRoad;

rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(std::llround(seconds * 1.0e9), RCL_ROS_TIME);
}

autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  autoware::vehicle_info_utils::VehicleInfo info;
  info.wheel_base_m = 2.74;
  info.wheel_tread_m = 1.63;
  info.front_overhang_m = 1.00;
  info.rear_overhang_m = 1.03;
  info.left_overhang_m = 0.10;
  info.right_overhang_m = 0.10;
  info.vehicle_height_m = 2.5;
  info.vehicle_length_m = 4.77;
  info.vehicle_width_m = 1.83;
  info.min_longitudinal_offset_m = -1.03;
  info.max_longitudinal_offset_m = 3.74;
  info.min_lateral_offset_m = -0.915;
  info.max_lateral_offset_m = 0.915;
  return info;
}

// Odometry every 0.05 s from 99 s to 106 s at (x(t), y(t)).
std::vector<Odometry> drive(
  const std::function<double(double)> & x_of, const std::function<double(double)> & y_of)
{
  std::vector<Odometry> odometry;
  for (int step = 0; 99.0 + step * 0.05 <= 106.0 + 1.0e-9; ++step) {
    const double time = 99.0 + step * 0.05;
    Odometry sample;
    sample.header.stamp = at(time);
    sample.pose.pose.position.x = x_of(time);
    sample.pose.pose.position.y = y_of(time);
    sample.pose.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    sample.twist.twist.linear.x = (x_of(time + 0.01) - x_of(time)) / 0.01;
    odometry.push_back(sample);
  }
  return odometry;
}

std::function<double(double)> steady(const double speed_mps)
{
  return [=](const double time) { return speed_mps * (time - 99.0); };
}

std::function<double(double)> ramp(
  const double from, const double to, const double start_s, const double end_s)
{
  return [=](const double time) {
    const double ratio = std::clamp((time - start_s) / (end_s - start_s), 0.0, 1.0);
    return from + ratio * (to - from);
  };
}

std::function<double(double)> constant(const double value)
{
  return [=](double) { return value; };
}

// A car standing at (x, y), logged every 0.1 s.
std::vector<TimedTrackedObjects> standing_car(const std::uint8_t id, const double x, const double y)
{
  std::vector<TimedTrackedObjects> frames;
  for (int step = 0; 99.55 + step * 0.1 <= 106.0 + 1.0e-9; ++step) {
    const double time = 99.55 + step * 0.1;
    auto objects = std::make_shared<TrackedObjects>();
    objects->header.stamp = at(time);
    autoware_perception_msgs::msg::TrackedObject object;
    object.object_id.uuid = std::array<std::uint8_t, 16>{id};
    object.kinematics.pose_with_covariance.pose.position.x = x;
    object.kinematics.pose_with_covariance.pose.position.y = y;
    object.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4.0;
    object.shape.dimensions.y = 1.8;
    ObjectClassification classification;
    classification.label = ObjectClassification::CAR;
    classification.probability = 1.0f;
    object.classification.push_back(classification);
    objects->objects.push_back(object);
    frames.push_back(TimedTrackedObjects{at(time), objects});
  }
  return frames;
}

// Out into the left lane between 100.5 s and 102 s, and back between 103.5 s and 104.5 s.
double swerve(const double time)
{
  return time < 103.5 ? ramp(0.0, 3.5, 100.5, 102.0)(time) : ramp(3.5, 0.0, 103.5, 104.5)(time);
}

std::vector<char> rows(const std::vector<int> & values)
{
  return std::vector<char>(values.begin(), values.end());
}

}  // namespace

TEST(LaneGoalRules, FirstHoldAndChooseLane)
{
  const auto inside = rows({0, 0, 1, 1, 1, 0});

  EXPECT_EQ(first_hold(inside, 0, 6, 2), 2U);
  EXPECT_FALSE(first_hold(inside, 0, 6, 4).has_value());
  EXPECT_FALSE(first_hold(inside, 3, 6, 3).has_value());
  EXPECT_EQ(choose_lane({rows({1, 1, 0}), rows({1, 0, 0})}, 0.5), 0U);
  EXPECT_FALSE(choose_lane({rows({1, 0, 0})}, 0.5).has_value());
  EXPECT_FALSE(choose_lane({}, 0.5).has_value());
}

TEST(LaneGoalRules, ALaneChangeReachesItsLaneOrSaysWhyNot)
{
  const EndRule end{0.6, 0.05, 2};
  const std::vector<double> none(6, 0.0);

  const auto reached =
    lane_change_goal(rows({0, 0, 0, 1, 1, 1}), rows({0, 0, 1, 1, 1, 1}), 6, 1, 2, end, none, none);
  EXPECT_EQ(reached.goal_row, 3U);
  EXPECT_EQ(reached.recorded_goal_row, 2U);
  EXPECT_EQ(reached.passed(), true);

  const auto over =
    lane_change_goal(rows({0, 0, 0, 1, 1, 1}), rows({1, 1, 1, 1, 1, 1}), 6, 1, 2, end, none, none);
  EXPECT_EQ(over.undecided, "recorded_in_target_lane");
  EXPECT_FALSE(over.passed().has_value());

  const auto failed =
    lane_change_goal(rows({0, 0, 0, 0, 0, 0}), rows({0, 0, 1, 1, 1, 1}), 6, 1, 2, end, none, none);
  EXPECT_EQ(failed.failure, "not_in_target_lane");
  EXPECT_EQ(failed.passed(), false);

  const auto ended =
    lane_change_goal(rows({0, 0, 0}), rows({0, 0, 1, 1, 1, 1}), 6, 1, 2, end, none, none);
  EXPECT_EQ(ended.undecided, "run_ended");

  // Not wholly in by the end, but 70 % of the box there, up from 40 % two rows before.
  const std::vector<double> moving{0.0, 0.1, 0.2, 0.4, 0.55, 0.7};
  const auto moving_in = lane_change_goal(
    rows({0, 0, 0, 0, 0, 0}), rows({0, 0, 1, 1, 1, 1}), 6, 1, 2, end, moving, none);
  EXPECT_TRUE(moving_in.moving_in);
  EXPECT_EQ(moving_in.goal_row, 5U);
}

TEST(LaneGoalRules, AnAvoidanceMustGetPastTheVehicleAndBackInItsLane)
{
  const EndRule end{0.6, 0.05, 2};
  const std::vector<double> none(6, 0.0);
  const std::vector<double> recorded_progress{0.0, 5.0, 10.0, 15.0, 20.0, 25.0};

  const auto passed = avoidance_goal(
    rows({1, 0, 0, 1, 1, 1}), rows({1, 0, 0, 1, 1, 1}), recorded_progress, recorded_progress, 12.0,
    6, 1, end, none, none);
  EXPECT_EQ(passed.progress_row, 3U);
  EXPECT_EQ(passed.goal_row, 3U);
  EXPECT_EQ(passed.passed(), true);

  const std::vector<double> stopped{0.0, 5.0, 8.0, 8.0, 8.0, 8.0};
  const auto behind = avoidance_goal(
    rows({1, 1, 1, 1, 1, 1}), rows({1, 0, 0, 1, 1, 1}), stopped, recorded_progress, 12.0, 6, 1, end,
    none, none);
  EXPECT_EQ(behind.failure, "not_past_vehicle");
  EXPECT_DOUBLE_EQ(*behind.max_progress_m, 8.0);

  const auto not_back = avoidance_goal(
    rows({1, 0, 0, 0, 0, 0}), rows({1, 0, 0, 1, 1, 1}), recorded_progress, recorded_progress, 12.0,
    6, 1, end, none, none);
  EXPECT_EQ(not_back.failure, "not_in_preferred_lane");
}

TEST(LaneGoal, AnEasyLaneChangeReachesTheTargetLaneWithItsWheels)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto lane_change = drive(steady(10.0), ramp(0.0, 3.5, 101.0, 103.0));
  const auto episode =
    build_closed_loop_episode(lane_change, {}, lane_change, at(100.0), at(104.0), at(104.0));

  const auto result = evaluate_lane_goal(episode, "lane_change", {}, layout, make_vehicle_info());

  EXPECT_TRUE(result.lane_found);
  EXPECT_EQ(result.verdict.passed(), true);
  // The wheels (0.815 m either side of the centre line) are wholly in the left lane widened to
  // 1.65 m once the ego is 2.465 m over: after 102.41 s, row 25.
  EXPECT_EQ(result.verdict.goal_row, 25U);
  EXPECT_EQ(result.verdict.recorded_goal_row, 25U);
}

TEST(LaneGoal, ARunThatStaysInItsLaneFailsTheLaneChange)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto episode = build_closed_loop_episode(
    drive(steady(10.0), constant(0.0)), {}, drive(steady(10.0), ramp(0.0, 3.5, 101.0, 103.0)),
    at(100.0), at(104.0), at(104.0));

  const auto result = evaluate_lane_goal(episode, "lane_change", {}, layout, make_vehicle_info());

  EXPECT_EQ(result.verdict.failure, "not_in_target_lane");
  ASSERT_EQ(result.fail_reasons.size(), 1U);
  EXPECT_NE(
    result.fail_reasons.front().find(
      "not wholly in the target lane from any row before the window's end at 4.0 s"),
    std::string::npos);
}

TEST(LaneGoal, AnEasyAvoidancePassesTheStandingCarAndComesBack)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto avoidance = drive(steady(10.0), swerve);
  const auto episode = build_closed_loop_episode(
    avoidance, standing_car(5, 40.0, 0.0), avoidance, at(100.0), at(105.0), at(105.0));

  const auto result = evaluate_lane_goal(episode, "avoidance", {}, layout, make_vehicle_info());

  ASSERT_TRUE(result.avoided.has_value());
  EXPECT_EQ(result.verdict.passed(), true);
  // Back with its wheels in its lane (widened to 1.85 m) once 1.035 m over: after 104.2 s.
  EXPECT_EQ(result.verdict.goal_row, 43U);
  EXPECT_NEAR(result.avoided->near_m, 3.5 - 0.915 - 0.9, 0.05);
}

TEST(LaneGoal, ARunThatStopsBehindTheCarNeverGetsPast)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto avoidance = drive(steady(10.0), swerve);
  const auto stops =
    drive([](const double time) { return std::min(10.0 * (time - 99.0), 30.0); }, constant(0.0));
  const auto episode = build_closed_loop_episode(
    stops, standing_car(5, 40.0, 0.0), avoidance, at(100.0), at(105.0), at(105.0));

  const auto result = evaluate_lane_goal(episode, "avoidance", {}, layout, make_vehicle_info());

  EXPECT_EQ(result.verdict.failure, "not_past_vehicle");
  EXPECT_EQ(result.verdict.passed(), false);
}

TEST(LaneGoal, RefusesAPullOut)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto straight = drive(steady(10.0), constant(0.0));
  const auto episode =
    build_closed_loop_episode(straight, {}, straight, at(100.0), at(104.0), at(104.0));

  EXPECT_THROW(
    evaluate_lane_goal(episode, "pull_out", {}, layout, make_vehicle_info()),
    std::invalid_argument);
}

}  // namespace autoware::planning_data_analyzer::closed_loop
