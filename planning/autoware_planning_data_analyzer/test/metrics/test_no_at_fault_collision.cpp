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

#include "metrics/epdms/subscores/no_at_fault_collision.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
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
  trajectory.header.stamp.sec = 10;
  trajectory.points.resize(2);
  trajectory.points[0].pose = make_pose(0.0, 0.0);
  trajectory.points[0].longitudinal_velocity_mps = speed_mps;
  trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  trajectory.points[1].pose = make_pose(speed_mps, 0.0);
  trajectory.points[1].longitudinal_velocity_mps = speed_mps;
  trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);
  return trajectory;
}

autoware_planning_msgs::msg::Trajectory make_stopped_then_moving_trajectory()
{
  auto trajectory = make_straight_trajectory(5.0);
  trajectory.points[0].longitudinal_velocity_mps = 0.0;
  return trajectory;
}

autoware_perception_msgs::msg::TrackedObject make_object(
  const double x, const double y, const std::uint8_t label, const std::array<uint8_t, 16> & uuid,
  const double speed_mps = 1.0)
{
  autoware_perception_msgs::msg::TrackedObject object;
  object.object_id.uuid = uuid;
  object.kinematics.pose_with_covariance.pose = make_pose(x, y);
  object.kinematics.twist_with_covariance.twist.linear.x = speed_mps;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.5;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0f;
  object.classification.push_back(classification);
  return object;
}

TimedTrackedObjects make_timed_objects(
  const rclcpp::Time & stamp,
  const std::vector<autoware_perception_msgs::msg::TrackedObject> & objects)
{
  auto tracked_objects = std::make_shared<TrackedObjects>();
  tracked_objects->header.stamp = stamp;
  tracked_objects->objects = objects;
  return TimedTrackedObjects{stamp, tracked_objects};
}

std::vector<TimedTrackedObjects> make_future_objects(
  const std::vector<autoware_perception_msgs::msg::TrackedObject> & objects)
{
  return {make_timed_objects(rclcpp::Time(10, 0, RCL_ROS_TIME), objects)};
}

}  // namespace

TEST(NoAtFaultCollision, EmptyObjectsPasses)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const std::vector<TimedTrackedObjects> future_objects{
    make_timed_objects(rclcpp::Time(10, 0, RCL_ROS_TIME), {})};
  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, FrontCollisionWithAgentFails)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const auto future_objects = make_future_objects({make_object(
    4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR,
    std::array<uint8_t, 16>{1})});

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
  EXPECT_GE(result.infraction_time_s, 0.0);
}

TEST(NoAtFaultCollision, FrontCollisionWithNonAgentGetsHalfPenalty)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const auto future_objects = make_future_objects({make_object(
    4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::HAZARD,
    std::array<uint8_t, 16>{2})});

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.5);
  EXPECT_EQ(result.reason, "at_fault_collision_with_non_agent");
}

TEST(NoAtFaultCollision, RearCollisionDoesNotFail)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const auto future_objects = make_future_objects({make_object(
    -3.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR,
    std::array<uint8_t, 16>{3})});

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, UsesLoggedFutureObjectAtMatchingTime)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const auto future_objects = std::vector<TimedTrackedObjects>{
    make_timed_objects(
      rclcpp::Time(10, 0, RCL_ROS_TIME),
      {make_object(
        20.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR,
        std::array<uint8_t, 16>{4})}),
    make_timed_objects(
      rclcpp::Time(11, 0, RCL_ROS_TIME),
      {make_object(
        8.8, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR,
        std::array<uint8_t, 16>{4})})};

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
}

TEST(NoAtFaultCollision, IgnoredCollisionDoesNotMaskLaterAtFaultCollisionWithSameObject)
{
  const auto trajectory = make_stopped_then_moving_trajectory();
  constexpr std::array<uint8_t, 16> uuid{6};
  const auto future_objects = std::vector<TimedTrackedObjects>{
    make_timed_objects(
      rclcpp::Time(10, 0, RCL_ROS_TIME),
      {make_object(4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, uuid)}),
    make_timed_objects(
      rclcpp::Time(11, 0, RCL_ROS_TIME),
      {make_object(8.8, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, uuid)})};

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
  EXPECT_DOUBLE_EQ(result.infraction_time_s, 1.0);
}

TEST(NoAtFaultCollision, UnknownObjectsAreIgnored)
{
  const auto trajectory = make_straight_trajectory(5.0);
  const auto future_objects = make_future_objects({make_object(
    4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::UNKNOWN,
    std::array<uint8_t, 16>{5})});

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

}  // namespace autoware::planning_data_analyzer::metrics
