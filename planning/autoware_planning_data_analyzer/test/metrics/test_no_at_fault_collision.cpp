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
#include <cmath>
#include <cstdint>
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

TEST(EgoPath, FromTrajectoryKeepsStampTimesPosesAndPlanarSpeed)
{
  auto trajectory = make_straight_trajectory(5.0);
  trajectory.points[1].longitudinal_velocity_mps = 5.0;
  trajectory.points[1].lateral_velocity_mps = 12.0;

  const auto path = to_ego_path(trajectory);

  EXPECT_EQ(path.start_time, rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_EQ(path.points.size(), 2U);
  EXPECT_EQ(path.points[1].time_from_start, rclcpp::Duration::from_seconds(1.0));
  EXPECT_DOUBLE_EQ(path.points[1].pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(path.points[0].speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(path.points[1].speed_mps, 13.0);
}

TEST(NoAtFaultCollision, EgoPathGivesTheTrajectoryResult)
{
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  using Case = std::pair<autoware_planning_msgs::msg::Trajectory, std::vector<TimedTrackedObjects>>;
  constexpr std::array<uint8_t, 16> uuid{6};
  const std::vector<Case> cases{
    {make_straight_trajectory(5.0),
     make_future_objects({make_object(4.0, 0.0, Classification::CAR, uuid)})},
    {make_straight_trajectory(5.0),
     make_future_objects({make_object(4.0, 0.0, Classification::HAZARD, uuid)})},
    {make_straight_trajectory(5.0),
     make_future_objects({make_object(-3.0, 0.0, Classification::CAR, uuid)})},
    {make_stopped_then_moving_trajectory(),
     {make_timed_objects(
        rclcpp::Time(10, 0, RCL_ROS_TIME), {make_object(4.0, 0.0, Classification::CAR, uuid)}),
      make_timed_objects(
        rclcpp::Time(11, 0, RCL_ROS_TIME), {make_object(8.8, 0.0, Classification::CAR, uuid)})}},
  };

  for (const auto & [trajectory, future_objects] : cases) {
    const auto expected = calculate_no_at_fault_collision(
      trajectory, future_objects, make_vehicle_info(), nullptr, true);
    const auto path = to_ego_path(trajectory);
    const auto actual = calculate_no_at_fault_collision(
      path, build_logged_object_tracks(future_objects), make_vehicle_info(), nullptr,
      evaluate_ego_path_footprints(path, make_vehicle_info()), true);

    EXPECT_EQ(actual.available, expected.available);
    EXPECT_DOUBLE_EQ(actual.score, expected.score);
    EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_DOUBLE_EQ(actual.infraction_time_s, expected.infraction_time_s);
    EXPECT_EQ(actual.debug_info.events.size(), expected.debug_info.events.size());
    EXPECT_EQ(
      actual.debug_info.ego_horizon_footprints.size(),
      expected.debug_info.ego_horizon_footprints.size());
  }
}

TEST(NoAtFaultCollision, DrivenPathIntoAStandingCarIsAtFault)
{
  // A closed-loop ego that drove 5 m/s for 2 s, sampled every 0.5 s, toward a car standing 10 m
  // ahead (its rear at 9 m).
  EgoPath path;
  path.start_time = rclcpp::Time(20, 0, RCL_ROS_TIME);
  for (int step = 0; step <= 4; ++step) {
    EgoPathPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5 * step);
    point.pose = make_pose(2.5 * step, 0.0);
    point.speed_mps = 5.0;
    path.points.push_back(point);
  }
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  constexpr std::array<uint8_t, 16> uuid{7};
  const std::vector<TimedTrackedObjects> objects{
    make_timed_objects(
      rclcpp::Time(20, 0, RCL_ROS_TIME), {make_object(10.0, 0.0, Classification::CAR, uuid, 0.0)}),
    make_timed_objects(
      rclcpp::Time(22, 0, RCL_ROS_TIME), {make_object(10.0, 0.0, Classification::CAR, uuid, 0.0)})};

  const auto result = calculate_no_at_fault_collision(
    path, build_logged_object_tracks(objects), make_vehicle_info(), nullptr,
    evaluate_ego_path_footprints(path, make_vehicle_info()));

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
  // The front bumper (3.74 m ahead of the pose) is at 8.74 m at 1.0 s and past 9 m at 1.5 s.
  EXPECT_DOUBLE_EQ(result.infraction_time_s, 1.5);
}

namespace
{

// A path driving +x at speed_mps, one point every interval_s from 10 s on.
EgoPath make_straight_path(const double speed_mps, const int points, const double interval_s = 0.5)
{
  EgoPath path;
  path.start_time = rclcpp::Time(10, 0, RCL_ROS_TIME);
  for (int step = 0; step < points; ++step) {
    EgoPathPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(interval_s * step);
    point.pose = make_pose(speed_mps * interval_s * step, 0.0);
    point.speed_mps = speed_mps;
    path.points.push_back(point);
  }
  return path;
}

// Footprints of the path with the given area flags on every point (no map needed).
std::vector<TrajectoryFootprintEvaluation> make_flagged_footprints(
  const EgoPath & path, const bool multiple_lanes, const bool non_drivable_area)
{
  auto evaluations = evaluate_ego_path_footprints(path, make_vehicle_info());
  for (auto & evaluation : evaluations) {
    EgoAreaEvaluation area;
    area.flags.multiple_lanes = multiple_lanes;
    area.flags.non_drivable_area = non_drivable_area;
    evaluation.ego_area_evaluation = area;
  }
  return evaluations;
}

std::vector<TimedTrackedObjects> make_still_objects(
  const std::vector<autoware_perception_msgs::msg::TrackedObject> & objects)
{
  return {
    make_timed_objects(rclcpp::Time(10, 0, RCL_ROS_TIME), objects),
    make_timed_objects(rclcpp::Time(13, 0, RCL_ROS_TIME), objects)};
}

}  // namespace

TEST(NoAtFaultCollisionOptions, FirstContactDecidesSettlesAnObjectAfterANotAtFaultContact)
{
  // The ego touches the car while stopped (not its fault), then drives into it.
  const auto trajectory = make_stopped_then_moving_trajectory();
  constexpr std::array<uint8_t, 16> uuid{6};
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  const std::vector<TimedTrackedObjects> future_objects{
    make_timed_objects(
      rclcpp::Time(10, 0, RCL_ROS_TIME), {make_object(4.0, 0.0, Classification::CAR, uuid)}),
    make_timed_objects(
      rclcpp::Time(11, 0, RCL_ROS_TIME), {make_object(8.8, 0.0, Classification::CAR, uuid)})};
  const auto path = to_ego_path(trajectory);
  NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;

  const auto result = calculate_no_at_fault_collision(
    path, build_logged_object_tracks(future_objects), make_vehicle_info(), nullptr,
    evaluate_ego_path_footprints(path, make_vehicle_info()), false, options);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
  ASSERT_EQ(result.contacts.size(), 1U);
  EXPECT_EQ(result.contacts[0].point_index, 0U);
  EXPECT_EQ(result.contacts[0].collision_type, "STOPPED_EGO");
  EXPECT_FALSE(result.contacts[0].at_fault);
}

TEST(NoAtFaultCollisionOptions, FirstContactDecidesKeepsAnAtFaultFirstContact)
{
  const auto path = make_straight_path(5.0, 3);
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;

  const auto result = calculate_no_at_fault_collision(
    path,
    build_logged_object_tracks(make_still_objects(
      {make_object(6.0, 0.0, Classification::CAR, std::array<uint8_t, 16>{8}, 0.0)})),
    make_vehicle_info(), nullptr, evaluate_ego_path_footprints(path, make_vehicle_info()), false,
    options);

  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
  // The front bumper reaches the car's rear (5 m) at the second point (2.5 + 3.74 m).
  EXPECT_DOUBLE_EQ(result.infraction_time_s, 0.5);
  ASSERT_EQ(result.contacts.size(), 1U);
  EXPECT_EQ(result.contacts[0].point_index, 1U);
  EXPECT_EQ(result.contacts[0].collision_type, "STOPPED_TRACK");
  EXPECT_TRUE(result.contacts[0].at_fault);
  EXPECT_FALSE(result.contacts[0].exempt);
}

TEST(NoAtFaultCollisionOptions, FromBehindExemptionExcusesACarOnTheRearCorner)
{
  // A standing car on the ego's rear-left corner, heading the ego's way: centre 1.5 m behind
  // the pose (the rear bumper is 1.03 m behind it) and 141 deg off the ego's heading, so the
  // NAVSIM rule calls it a stopped track, which is the ego's fault.
  const auto path = make_straight_path(5.0, 1);
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  const auto tracks = build_logged_object_tracks(make_still_objects(
    {make_object(-1.5, 1.2, Classification::CAR, std::array<uint8_t, 16>{9}, 0.0)}));
  const auto footprints = evaluate_ego_path_footprints(path, make_vehicle_info());

  const auto without = calculate_no_at_fault_collision(
    path, tracks, make_vehicle_info(), nullptr, footprints, false, {});
  NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;
  options.from_behind_heading_deg = 45.0;
  const auto with = calculate_no_at_fault_collision(
    path, tracks, make_vehicle_info(), nullptr, footprints, false, options);

  EXPECT_DOUBLE_EQ(without.score, 0.0);
  EXPECT_DOUBLE_EQ(with.score, 1.0);
  ASSERT_EQ(with.contacts.size(), 1U);
  EXPECT_FALSE(with.contacts[0].at_fault);
  EXPECT_TRUE(with.contacts[0].exempt);
}

TEST(NoAtFaultCollisionOptions, FromBehindExemptionNeedsTheEgosHeading)
{
  // The same corner contact with a car crossing at 90 deg is not excused.
  const auto path = make_straight_path(5.0, 1);
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  auto object = make_object(-1.5, 1.2, Classification::CAR, std::array<uint8_t, 16>{9}, 0.0);
  object.kinematics.pose_with_covariance.pose = make_pose(-1.5, 1.2, M_PI_2);
  NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;
  options.from_behind_heading_deg = 45.0;

  const auto result = calculate_no_at_fault_collision(
    path, build_logged_object_tracks(make_still_objects({object})), make_vehicle_info(), nullptr,
    evaluate_ego_path_footprints(path, make_vehicle_info()), false, options);

  EXPECT_DOUBLE_EQ(result.score, 0.0);
  ASSERT_EQ(result.contacts.size(), 1U);
  EXPECT_TRUE(result.contacts[0].at_fault);
  EXPECT_FALSE(result.contacts[0].exempt);
}

TEST(NoAtFaultCollisionOptions, ExcusedStraddlingIsNotTheEgosFault)
{
  // A moving car alongside the ego's left side: a lateral contact.
  const auto path = make_straight_path(5.0, 1);
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  const auto tracks = build_logged_object_tracks(make_still_objects(
    {make_object(1.5, 1.4, Classification::CAR, std::array<uint8_t, 16>{10}, 5.0)}));
  NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;
  options.straddling_excused = {true};

  const auto straddling = make_flagged_footprints(path, true, false);
  const auto plain = calculate_no_at_fault_collision(
    path, tracks, make_vehicle_info(), nullptr, straddling, false, {});
  const auto excused = calculate_no_at_fault_collision(
    path, tracks, make_vehicle_info(), nullptr, straddling, false, options);
  const auto off_road = calculate_no_at_fault_collision(
    path, tracks, make_vehicle_info(), nullptr, make_flagged_footprints(path, true, true), false,
    options);

  EXPECT_DOUBLE_EQ(plain.score, 0.0);
  EXPECT_EQ(plain.reason, "at_fault_lateral_collision_with_agent");
  EXPECT_DOUBLE_EQ(excused.score, 1.0);
  ASSERT_EQ(excused.contacts.size(), 1U);
  EXPECT_EQ(excused.contacts[0].collision_type, "ACTIVE_LATERAL");
  EXPECT_TRUE(excused.contacts[0].multiple_lanes);
  EXPECT_TRUE(excused.contacts[0].straddling_excused);
  EXPECT_FALSE(excused.contacts[0].at_fault);
  EXPECT_DOUBLE_EQ(off_road.score, 0.0);
}

TEST(NoAtFaultCollisionOptions, ExcusedFlagsMustMatchThePath)
{
  const auto path = make_straight_path(5.0, 2);
  NoAtFaultCollisionOptions options;
  options.straddling_excused = {true};

  const auto result = calculate_no_at_fault_collision(
    path, {}, make_vehicle_info(), nullptr, evaluate_ego_path_footprints(path, make_vehicle_info()),
    false, options);

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.reason, "unavailable_invalid_straddling_excused");
}

}  // namespace autoware::planning_data_analyzer::metrics
