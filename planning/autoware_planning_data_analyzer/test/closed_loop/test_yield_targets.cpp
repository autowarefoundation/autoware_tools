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
#include "closed_loop/yield_targets.hpp"
#include "metrics/geometry/metric_utils.hpp"
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

// Road users moving at (vx, vy) from (x0, y0) at 100 s, logged every 0.1 s.
struct Mover
{
  std::uint8_t id;
  std::uint8_t label;
  double x0;
  double y0;
  double vx;
  double vy;
  double length_m;
  double width_m;
};

std::vector<TimedTrackedObjects> movers(const std::vector<Mover> & all)
{
  std::vector<TimedTrackedObjects> frames;
  for (int step = 0; 99.05 + step * 0.1 <= 110.0 + 1.0e-9; ++step) {
    const double time = 99.05 + step * 0.1;
    auto objects = std::make_shared<TrackedObjects>();
    objects->header.stamp = at(time);
    for (const auto & mover : all) {
      autoware_perception_msgs::msg::TrackedObject object;
      object.object_id.uuid = std::array<std::uint8_t, 16>{mover.id};
      object.kinematics.pose_with_covariance.pose.position.x = mover.x0 + mover.vx * (time - 100.0);
      object.kinematics.pose_with_covariance.pose.position.y = mover.y0 + mover.vy * (time - 100.0);
      object.kinematics.pose_with_covariance.pose.orientation =
        autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(mover.vy, mover.vx));
      object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      object.shape.dimensions.x = mover.length_m;
      object.shape.dimensions.y = mover.width_m;
      ObjectClassification classification;
      classification.label = mover.label;
      classification.probability = 1.0f;
      object.classification.push_back(classification);
      objects->objects.push_back(object);
    }
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

TargetPick pick(
  const std::string & kind, const std::vector<Mover> & all, const double ego_speed_mps)
{
  static const StraightRoad road;
  const auto ego = drive(ego_speed_mps);
  const auto episode =
    build_closed_loop_episode(ego, movers(all), ego, at(100.0), at(108.0), at(108.0));
  return pick_targets(episode, kind, road.map(), make_vehicle_info(), {});
}

bool picked(const TargetPick & pick, const std::uint8_t id)
{
  return std::find(pick.target_track_ids.begin(), pick.target_track_ids.end(), id_of(id)) !=
         pick.target_track_ids.end();
}

}  // namespace

TEST(YieldTargets, ThePedestrianTheHumanWaitedForIsTheFinalTarget)
{
  // A pedestrian crossing at 20 m before the human (3 m/s) gets there; another walking along the
  // far kerb never comes near the human's path.
  const auto result = pick(
    "pedestrian_yield",
    {{1, ObjectClassification::PEDESTRIAN, 20.0, -6.0, 0.0, 2.0, 0.6, 0.6},
     {2, ObjectClassification::PEDESTRIAN, 0.0, 12.0, 1.0, 0.0, 0.6, 0.6}},
    3.0);

  ASSERT_TRUE(result.final_track_id.has_value());
  EXPECT_EQ(*result.final_track_id, id_of(1));
  EXPECT_TRUE(picked(result, 1));
  EXPECT_FALSE(picked(result, 2));
}

TEST(YieldTargets, AVehicleYieldPicksTheCrossingCarNotThePedestrian)
{
  // A car crossing at 15 m before the human, and a pedestrian crossing at 20 m.
  const auto result = pick(
    "vehicle_yield",
    {{3, ObjectClassification::CAR, 15.0, -10.0, 0.0, 5.0, 4.0, 1.8},
     {1, ObjectClassification::PEDESTRIAN, 20.0, -6.0, 0.0, 2.0, 0.6, 0.6}},
    3.0);

  ASSERT_TRUE(result.final_track_id.has_value());
  EXPECT_EQ(*result.final_track_id, id_of(3));
  EXPECT_EQ(result.reasons.at(id_of(3)).front(), "final");
}

TEST(YieldTargets, RefusesAnotherKind)
{
  static const StraightRoad road;
  const auto ego = drive(3.0);
  const auto episode = build_closed_loop_episode(ego, {}, ego, at(100.0), at(108.0), at(108.0));

  EXPECT_THROW(
    pick_targets(episode, "lane_change", road.map(), make_vehicle_info(), {}),
    std::invalid_argument);
  EXPECT_TRUE(pick_targets(episode, "pedestrian_yield", road.map(), make_vehicle_info(), {})
                .target_track_ids.empty());
}

}  // namespace autoware::planning_data_analyzer::closed_loop
