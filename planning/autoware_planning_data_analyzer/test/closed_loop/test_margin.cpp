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
#include "closed_loop/lane_layout.hpp"
#include "closed_loop/margin.hpp"
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

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

// An ego driving +x at speed_mps from x = 0 at 99 s, at y(t); odometry every 0.05 s to 106 s.
std::vector<Odometry> drive(
  const double speed_mps, const std::function<double(double)> & y_of = nullptr)
{
  std::vector<Odometry> odometry;
  for (int step = 0; 99.0 + step * 0.05 <= 106.0 + 1.0e-9; ++step) {
    const double time = 99.0 + step * 0.05;
    Odometry sample;
    sample.header.stamp = at(time);
    sample.pose.pose = make_pose(speed_mps * (time - 99.0), y_of ? y_of(time) : 0.0);
    sample.twist.twist.linear.x = speed_mps;
    odometry.push_back(sample);
  }
  return odometry;
}

// y ramping from `from` to `to` over [start_s, end_s].
std::function<double(double)> ramp(
  const double from, const double to, const double start_s, const double end_s)
{
  return [=](const double time) {
    const double ratio = std::clamp((time - start_s) / (end_s - start_s), 0.0, 1.0);
    return from + ratio * (to - from);
  };
}

// A car logged every 0.1 s from 99.55 s to 106 s at x0 + speed * (t - 99.5), y.
std::vector<TimedTrackedObjects> car_track(
  const std::uint8_t id, const double x0, const double y, const double speed_mps,
  std::vector<TimedTrackedObjects> frames = {})
{
  std::vector<TimedTrackedObjects> merged;
  for (int step = 0; 99.55 + step * 0.1 <= 106.0 + 1.0e-9; ++step) {
    const double time = 99.55 + step * 0.1;
    auto objects = std::make_shared<TrackedObjects>();
    objects->header.stamp = at(time);
    if (static_cast<std::size_t>(step) < frames.size()) {
      *objects = *frames.at(static_cast<std::size_t>(step)).objects;
    }
    autoware_perception_msgs::msg::TrackedObject object;
    object.object_id.uuid = std::array<std::uint8_t, 16>{id};
    object.kinematics.pose_with_covariance.pose = make_pose(x0 + speed_mps * (time - 99.5), y);
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4.0;
    object.shape.dimensions.y = 1.8;
    object.shape.dimensions.z = 1.5;
    ObjectClassification classification;
    classification.label = ObjectClassification::CAR;
    classification.probability = 1.0f;
    object.classification.push_back(classification);
    objects->objects.push_back(object);
    merged.push_back(TimedTrackedObjects{at(time), objects});
  }
  return merged;
}

// The run is the recorded drive itself (a GT run) over the window [100 s, end_s].
ClosedLoopEpisode episode_of(
  const std::vector<Odometry> & odometry, const std::vector<TimedTrackedObjects> & objects,
  const double end_s = 104.0)
{
  return build_closed_loop_episode(odometry, objects, odometry, at(100.0), at(end_s), at(end_s));
}

MarginReference reference_of(const std::string & kind)
{
  MarginReference reference;
  reference.kind = kind;
  return reference;
}

const MarginRecord * record_of(const MarginResult & result, const std::string & role)
{
  const auto found = std::find_if(
    result.road_users.begin(), result.road_users.end(),
    [&](const auto & record) { return record.role == role; });
  return found == result.road_users.end() ? nullptr : &*found;
}

}  // namespace

TEST(Margin, ALaneChangeMeasuresTheRoadUsersOnceOutOfItsStartLane)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  // Into the left lane between 101 s and 103 s; a car 20 m ahead in that lane, at the ego's speed.
  const auto episode =
    episode_of(drive(10.0, ramp(0.0, 3.5, 101.0, 103.0)), car_track(1, 25.0, 3.5, 10.0));

  const auto result =
    evaluate_margin(episode, reference_of("lane_change"), layout, make_vehicle_info());

  EXPECT_TRUE(result.passed);
  EXPECT_TRUE(result.start_lane);
  const auto * record = record_of(result, "road_user");
  ASSERT_NE(record, nullptr);
  // Its rear is 20 - 2 = 18 m ahead of the ego's pose, the ego's front 3.74 m ahead of it.
  EXPECT_NEAR(record->distance_m, 14.26, 0.05);
  // Not before the ego leaves its lane (1.85 m at its left edge, after 101.27 s).
  EXPECT_GE(record->row, 13U);
}

TEST(Margin, InItsLaneALaneChangeMeasuresNothingButAnEasySceneDoes)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto episode = episode_of(drive(10.0), car_track(1, 25.0, 3.5, 10.0));
  auto easy = reference_of("lane_change");
  easy.every_road_user = true;

  EXPECT_TRUE(evaluate_margin(episode, reference_of("lane_change"), layout, make_vehicle_info())
                .road_users.empty());
  const auto result = evaluate_margin(episode, easy, layout, make_vehicle_info());
  ASSERT_NE(record_of(result, "road_user"), nullptr);
  EXPECT_TRUE(result.passed);
}

TEST(Margin, AnAtFaultCollisionFailsAndSaysWhy)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  // A car standing in the ego's lane, its rear at 38 m: the ego's front gets there at 102.43 s.
  const auto episode = episode_of(drive(10.0), car_track(2, 40.0, 0.0, 0.0));

  const auto result =
    evaluate_margin(episode, reference_of("lane_change"), layout, make_vehicle_info());

  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.at_fault_collision_count, 1U);
  ASSERT_EQ(result.contacts.size(), 1U);
  EXPECT_EQ(result.contacts.front().row, 25U);
  EXPECT_EQ(result.contacts.front().kind, "road user stopped");
  ASSERT_EQ(result.fail_reasons.size(), 1U);
  EXPECT_NE(
    result.fail_reasons.front().find("at 2.5 s: the ego's fault (road user stopped)"),
    std::string::npos);
}

TEST(Margin, ARoadUserDrivingDeepIntoTheEgoStopsTheJudging)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  // A car 7 m/s faster drives into the ego's rear at 101.14 s (row 12) and on into it; then the
  // ego reaches a car standing with its rear at 18.5 m at 101.95 s (row 20), after the judging
  // stopped.
  auto objects = car_track(3, -12.0, 0.0, 12.0);
  objects = car_track(4, 20.5, 0.0, 0.0, objects);
  const auto episode = episode_of(drive(5.0), objects);

  const auto result =
    evaluate_margin(episode, reference_of("lane_change"), layout, make_vehicle_info());

  ASSERT_TRUE(result.judged_until_row.has_value());
  EXPECT_EQ(*result.judged_until_row, 12U);
  EXPECT_TRUE(result.passed);
  ASSERT_EQ(result.contacts.size(), 2U);
  EXPECT_FALSE(result.contacts.at(0).at_fault);
  EXPECT_TRUE(result.contacts.at(1).at_fault);
  EXPECT_FALSE(result.contacts.at(1).judged);
}

TEST(Margin, AnAvoidanceHoldsAStandingVehicleToTheAvoidedObjectsLine)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  // Out into the left lane between 100.5 s and 102 s, past a car standing at 40 m, and back
  // between 103.5 s and 104.5 s.
  const auto y_of = [](const double time) {
    return time < 103.5 ? ramp(0.0, 3.5, 100.5, 102.0)(time) : ramp(3.5, 0.0, 103.5, 104.5)(time);
  };
  const auto episode = episode_of(drive(10.0, y_of), car_track(5, 40.0, 0.0, 0.0), 105.0);

  const auto result =
    evaluate_margin(episode, reference_of("avoidance"), layout, make_vehicle_info());

  EXPECT_TRUE(result.passed);
  const auto * record = record_of(result, "avoided_object");
  ASSERT_NE(record, nullptr);
  // Alongside: the ego's right side at 3.5 - 0.915 m, the car's left side at 0.9 m.
  EXPECT_NEAR(record->distance_m, 3.5 - 0.915 - 0.9, 0.05);
}

TEST(Margin, RefusesAnUnknownKindAndAMisplacedTrailingVehicle)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  const auto episode = episode_of(drive(10.0), {});
  auto trailing = reference_of("lane_change");
  trailing.trailing_vehicle = true;

  EXPECT_THROW(
    evaluate_margin(episode, reference_of("merge"), layout, make_vehicle_info()),
    std::invalid_argument);
  EXPECT_THROW(
    evaluate_margin(episode, trailing, layout, make_vehicle_info()), std::invalid_argument);
}

}  // namespace autoware::planning_data_analyzer::closed_loop
