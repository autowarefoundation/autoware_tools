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
#include "closed_loop/conflict.hpp"
#include "closed_loop/lane_layout.hpp"
#include "closed_loop/yield_crossing.hpp"
#include "metrics/geometry/metric_utils.hpp"
#include "straight_road.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
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

// An ego driving +x along y = 0 at speed_mps from x = 0 at 100 s, odometry every 0.05 s.
std::vector<Odometry> drive(const double speed_mps)
{
  std::vector<Odometry> odometry;
  for (int step = 0; 99.0 + step * 0.05 <= 110.0 + 1.0e-9; ++step) {
    const double time = 99.0 + step * 0.05;
    Odometry sample;
    sample.header.stamp = at(time);
    sample.pose.pose.position.x = speed_mps * (time - 100.0);
    sample.pose.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    sample.twist.twist.linear.x = speed_mps;
    odometry.push_back(sample);
  }
  return odometry;
}

// A road user crossing the road along +y at x, from y0 at 100 s at speed_mps.
std::vector<TimedTrackedObjects> crossing(
  const std::uint8_t id, const std::uint8_t label, const double x, const double y0,
  const double speed_mps, const double length_m, const double width_m)
{
  std::vector<TimedTrackedObjects> frames;
  for (int step = 0; 99.05 + step * 0.1 <= 110.0 + 1.0e-9; ++step) {
    const double time = 99.05 + step * 0.1;
    auto objects = std::make_shared<TrackedObjects>();
    objects->header.stamp = at(time);
    autoware_perception_msgs::msg::TrackedObject object;
    object.object_id.uuid = std::array<std::uint8_t, 16>{id};
    object.kinematics.pose_with_covariance.pose.position.x = x;
    object.kinematics.pose_with_covariance.pose.position.y = y0 + speed_mps * (time - 100.0);
    object.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(M_PI_2);
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = length_m;
    object.shape.dimensions.y = width_m;
    ObjectClassification classification;
    classification.label = label;
    classification.probability = 1.0f;
    object.classification.push_back(classification);
    objects->objects.push_back(object);
    frames.push_back(TimedTrackedObjects{at(time), objects});
  }
  return frames;
}

std::string id_of(const std::uint8_t id)
{
  unique_identifier_msgs::msg::UUID uuid;
  uuid.uuid = std::array<std::uint8_t, 16>{id};
  return metrics::object_id_to_string(uuid, true);
}

// A GT-like run: the executed ego is also the recorded drive, over [100 s, 108 s].
YieldResult judge(
  const double ego_speed_mps, const std::vector<TimedTrackedObjects> & objects,
  const std::vector<std::string> & targets)
{
  static const StraightRoad road;
  static const LaneLayout layout(road.map());
  const auto ego = drive(ego_speed_mps);
  const auto episode =
    build_closed_loop_episode(ego, objects, ego, at(100.0), at(108.0), at(108.0));
  return evaluate_yield(episode, targets, {}, layout, make_vehicle_info());
}

}  // namespace

TEST(Yield, APedestrianThatCrossedWellBeforeTheEgoPasses)
{
  // The pedestrian leaves the ego's lane at about 103.6 s; the ego (3 m/s) gets there at 105.3 s.
  const auto result = judge(
    3.0, crossing(1, ObjectClassification::PEDESTRIAN, 20.0, -6.0, 2.0, 0.6, 0.6), {id_of(1)});

  ASSERT_EQ(result.targets.size(), 1U);
  const auto & target = result.targets.front();
  ASSERT_TRUE(target.crossing.has_value());
  EXPECT_EQ(target.crossing->order, CrossingOrder::ROAD_USER_FIRST);
  EXPECT_EQ(target.measure, "pet");
  EXPECT_GE(*target.value_s, 1.3);
  EXPECT_EQ(target.passed, true);
  EXPECT_TRUE(result.passed);
}

TEST(Yield, AnEgoThatPassesRightBehindThePedestrianFailsOnPet)
{
  // At 4 m/s the ego gets there at 104.0 s, under 0.5 s after the pedestrian left.
  const auto result = judge(
    4.0, crossing(1, ObjectClassification::PEDESTRIAN, 20.0, -6.0, 2.0, 0.6, 0.6), {id_of(1)});

  ASSERT_EQ(result.targets.size(), 1U);
  const auto & target = result.targets.front();
  EXPECT_EQ(target.measure, "pet");
  ASSERT_TRUE(target.value_s.has_value());
  EXPECT_LT(*target.value_s, 1.3);
  EXPECT_EQ(target.passed, false);
  EXPECT_FALSE(result.passed);
  ASSERT_EQ(result.fail_reasons.size(), 1U);
  EXPECT_NE(result.fail_reasons.front().find("(pedestrian) went first"), std::string::npos);
}

TEST(Yield, AnEgoGoingFirstIsJudgedOnTheEntryGap)
{
  // The ego (5 m/s) enters the car's path at about 103.1 s; the car reaches it 2.0 s or 2.6 s
  // later.
  const auto close =
    judge(5.0, crossing(2, ObjectClassification::CAR, 20.0, -18.0, 3.0, 4.0, 1.8), {id_of(2)});
  const auto clear =
    judge(5.0, crossing(2, ObjectClassification::CAR, 20.0, -20.0, 3.0, 4.0, 1.8), {id_of(2)});

  ASSERT_TRUE(close.targets.front().crossing.has_value());
  EXPECT_EQ(close.targets.front().crossing->order, CrossingOrder::EGO_FIRST);
  EXPECT_EQ(close.targets.front().measure, "entry_gap");
  EXPECT_EQ(close.targets.front().passed, false);
  ASSERT_EQ(close.fail_reasons.size(), 1U);
  EXPECT_NE(close.fail_reasons.front().find("the ego went first ahead of"), std::string::npos);
  EXPECT_EQ(clear.targets.front().passed, true);
  EXPECT_TRUE(clear.passed);
}

TEST(Yield, ATargetTheEgoHitsIsACollisionAndNotMeasured)
{
  // The ego (5 m/s) drives into the pedestrian walking across at its arrival.
  const auto result = judge(
    5.0, crossing(1, ObjectClassification::PEDESTRIAN, 20.0, -4.0, 1.2, 0.6, 0.6), {id_of(1)});

  EXPECT_FALSE(result.passed);
  EXPECT_EQ(result.targets.front().unjudged, "the ego hit it, at fault");
  ASSERT_FALSE(result.contacts.empty());
  EXPECT_TRUE(result.contacts.front().at_fault);
  EXPECT_TRUE(result.contacts.front().target);
}

TEST(Yield, AnUnseenTargetIsNotJudged)
{
  const auto result = judge(3.0, {}, {id_of(9)});

  EXPECT_EQ(result.targets.front().unjudged, "not observed in the episode");
  EXPECT_FALSE(result.targets.front().passed.has_value());
  EXPECT_TRUE(result.passed);
}

}  // namespace autoware::planning_data_analyzer::closed_loop
