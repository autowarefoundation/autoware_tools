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

#include "metrics/epdms/subscores/ttc_within_bound.hpp"
#include "metrics/geometry/ego_footprint.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{
namespace
{

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

autoware_planning_msgs::msg::Trajectory make_straight_trajectory(const double speed_mps)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.points.resize(2);
  trajectory.points[0].pose = make_pose(0.0, 0.0);
  trajectory.points[0].longitudinal_velocity_mps = speed_mps;
  trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  trajectory.points[1].pose = make_pose(speed_mps, 0.0);
  trajectory.points[1].longitudinal_velocity_mps = speed_mps;
  trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);
  return trajectory;
}

autoware_perception_msgs::msg::TrackedObject make_stationary_object(const double x, const double y)
{
  autoware_perception_msgs::msg::TrackedObject object;
  object.kinematics.pose_with_covariance.pose = make_pose(x, y);
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.5;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0f;
  object.classification.push_back(classification);

  return object;
}

autoware_perception_msgs::msg::TrackedObject make_box_object(
  const double x, const double y, const unique_identifier_msgs::msg::UUID & object_id)
{
  auto object = make_stationary_object(x, y);
  object.object_id = object_id;
  return object;
}

builtin_interfaces::msg::Time make_stamp(const double stamp_s)
{
  const auto stamp_ns = static_cast<int64_t>(std::llround(stamp_s * 1.0e9));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000);
  return stamp;
}

std::vector<TimedTrackedObjects> make_future_objects(
  std::vector<autoware_perception_msgs::msg::TrackedObject> objects, const double stamp_s = 0.0)
{
  auto msg = std::make_shared<TrackedObjects>();
  msg->header.stamp = make_stamp(stamp_s);
  msg->objects = std::move(objects);
  return {TimedTrackedObjects{rclcpp::Time(msg->header.stamp), msg}};
}

unique_identifier_msgs::msg::UUID make_uuid(const std::array<uint8_t, 16> & bytes)
{
  unique_identifier_msgs::msg::UUID id;
  id.uuid = bytes;
  return id;
}

std::vector<TrajectoryFootprintEvaluation> make_footprint_evaluations(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const bool multiple_lanes,
  const bool non_drivable_area)
{
  std::vector<TrajectoryFootprintEvaluation> evaluations;
  evaluations.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    TrajectoryFootprintEvaluation evaluation;
    evaluation.ego_polygon = create_pose_footprint(point.pose, vehicle_info);
    EgoAreaEvaluation area;
    area.flags.multiple_lanes = multiple_lanes;
    area.flags.non_drivable_area = non_drivable_area;
    evaluation.ego_area_evaluation = area;
    evaluations.push_back(std::move(evaluation));
  }
  return evaluations;
}

}  // namespace

TEST(TTCWithinBound, EmptyObjectsPasses)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(TTCWithinBound, AheadCollisionFails)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  objects->objects.push_back(make_stationary_object(4.0, 0.0));

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "collision_within_bound");
  EXPECT_GE(result.infraction_time_s, 0.0);
}

TEST(TTCWithinBound, UnknownCollisionIsIgnored)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  auto object = make_stationary_object(4.0, 0.0);
  object.classification.front().label =
    autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  objects->objects.push_back(object);

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(TTCWithinBound, BehindCollisionDoesNotFail)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  objects->objects.push_back(make_stationary_object(-4.0, 0.0));

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(TTCWithinBound, UsesTrackedObjectPose)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  objects->objects.push_back(make_stationary_object(4.0, 0.0));

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "collision_within_bound");
}

TEST(TTCWithinBound, NuplanAheadAngleDoesNotFailForLargeLateralOffset)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  objects->objects.push_back(make_stationary_object(1.0, 1.2));

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(TTCWithinBound, BadAreaAllowsLateralOverlapToFail)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  objects->objects.push_back(make_stationary_object(1.0, 1.2));
  const auto evaluations = make_footprint_evaluations(trajectory, make_vehicle_info(), false, true);

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info(), nullptr, &evaluations);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "collision_within_bound");
}

TEST(TTCWithinBound, BadAreaBehindCollisionDoesNotFail)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<TrackedObjects>();
  objects->objects.push_back(make_stationary_object(-4.0, 0.0));
  const auto evaluations = make_footprint_evaluations(trajectory, make_vehicle_info(), false, true);

  const auto result = calculate_ttc_within_bound(
    trajectory, make_future_objects(objects->objects), make_vehicle_info(), nullptr, &evaluations);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(TTCWithinBound, PreviouslyCollidedObjectIsIgnored)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const auto object_id = make_uuid({1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1});

  auto objects_t0 = std::make_shared<TrackedObjects>();
  objects_t0->header.stamp = make_stamp(0.0);
  objects_t0->objects.push_back(make_box_object(-1.5, 0.0, object_id));

  auto objects_t1 = std::make_shared<TrackedObjects>();
  objects_t1->header.stamp = make_stamp(1.0);
  objects_t1->objects.push_back(make_box_object(6.0, 0.0, object_id));

  const std::vector<TimedTrackedObjects> future_objects = {
    TimedTrackedObjects{rclcpp::Time(objects_t0->header.stamp), objects_t0},
    TimedTrackedObjects{rclcpp::Time(objects_t1->header.stamp), objects_t1}};

  const auto result = calculate_ttc_within_bound(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

}  // namespace autoware::planning_data_analyzer::metrics
