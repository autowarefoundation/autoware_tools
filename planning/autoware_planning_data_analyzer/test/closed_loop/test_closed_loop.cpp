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

#include "closed_loop/closed_loop_bag_reader.hpp"
#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/closed_loop_evaluator.hpp"
#include "closed_loop/closed_loop_scene.hpp"
#include "metrics/geometry/object_tracks.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

using autoware_perception_msgs::msg::ObjectClassification;

constexpr std::int64_t kSecond = 1'000'000'000;

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

// Odometry every step_s over [start_s, end_s] of an ego driving +x at speed_mps from x0.
std::vector<Odometry> drive(
  const double start_s, const double end_s, const double speed_mps, const double x0 = 0.0,
  const double step_s = 0.05)
{
  std::vector<Odometry> odometry;
  for (int step = 0; start_s + step * step_s <= end_s + 1.0e-9; ++step) {
    const double time = start_s + step * step_s;
    Odometry sample;
    sample.header.stamp = at(time);
    sample.pose.pose = make_pose(x0 + speed_mps * (time - start_s), 0.0);
    sample.twist.twist.linear.x = speed_mps;
    odometry.push_back(sample);
  }
  return odometry;
}

autoware_perception_msgs::msg::TrackedObject make_object(
  const std::uint8_t id, const std::uint8_t label, const geometry_msgs::msg::Pose & pose,
  const double logged_speed_mps = 0.0)
{
  autoware_perception_msgs::msg::TrackedObject object;
  object.object_id.uuid = std::array<std::uint8_t, 16>{id};
  object.kinematics.pose_with_covariance.pose = pose;
  object.kinematics.twist_with_covariance.twist.linear.x = logged_speed_mps;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 1.8;
  object.shape.dimensions.z = 1.5;
  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0f;
  object.classification.push_back(classification);
  return object;
}

// A road user logged every 0.1 s (stamps start_s + 0.05 + k * 0.1) over [start_s, end_s].
std::vector<TimedTrackedObjects> track(
  const std::uint8_t id, const std::uint8_t label, const double start_s, const double end_s,
  const double x0, const double y, const double speed_mps, const double logged_speed_mps = 0.0)
{
  std::vector<TimedTrackedObjects> frames;
  for (int step = 0; start_s + 0.05 + step * 0.1 <= end_s + 1.0e-9; ++step) {
    const double time = start_s + 0.05 + step * 0.1;
    auto objects = std::make_shared<TrackedObjects>();
    objects->header.stamp = at(time);
    objects->objects.push_back(
      make_object(id, label, make_pose(x0 + speed_mps * (time - start_s), y), logged_speed_mps));
    frames.push_back(TimedTrackedObjects{at(time), objects});
  }
  return frames;
}

std::vector<TimedTrackedObjects> merge(
  std::vector<TimedTrackedObjects> lhs, const std::vector<TimedTrackedObjects> & rhs)
{
  lhs.insert(lhs.end(), rhs.begin(), rhs.end());
  std::stable_sort(
    lhs.begin(), lhs.end(), [](const auto & a, const auto & b) { return a.stamp < b.stamp; });
  return lhs;
}

nlohmann::json scene_json()
{
  return {
    {"schema_version", 1},
    {"scene_id", "dataset/2026-11-06/13-59-17@1793941903441943903"},
    {"eval_label", "lane_change"},
    {"difficulty", "easy"},
    {"window", {{"start_ns", 1793941903441943903}, {"end_ns", 1793941913441979467}}},
  };
}

// A scene whose label has no metric of its own: judged on NC alone.
nlohmann::json scene_json_without_label()
{
  auto json = scene_json();
  json["eval_label"] = "centerline";
  return json;
}

}  // namespace

TEST(ClosedLoopScene, ParsesAWindowAndDefaultsTheRunEndToIt)
{
  const auto scene = parse_closed_loop_scene(scene_json());

  EXPECT_EQ(scene.eval_label, "lane_change");
  EXPECT_EQ(scene.difficulty, "easy");
  EXPECT_EQ(scene.window_start.nanoseconds(), 1793941903441943903);
  EXPECT_EQ(scene.run_end.nanoseconds(), scene.window_end.nanoseconds());
  EXPECT_TRUE(scene.reference_bag_path.empty());
  EXPECT_EQ(scene.reference_time_offset_ns, 0);
  EXPECT_EQ(parse_closed_loop_scene(to_json(scene)).window_end, scene.window_end);
}

TEST(ClosedLoopScene, ReadsTheReferenceDriveAndAnUnratedScene)
{
  auto json = scene_json();
  json["difficulty"] = nullptr;
  json["run_end_ns"] = 1793941915000000000;
  json["reference"] = {{"bag_path", "/data/recorded"}, {"time_offset_ns", -42}};

  const auto scene = parse_closed_loop_scene(json);

  EXPECT_TRUE(scene.difficulty.empty());
  EXPECT_EQ(scene.run_end.nanoseconds(), 1793941915000000000);
  EXPECT_EQ(scene.reference_bag_path, "/data/recorded");
  EXPECT_EQ(scene.reference_time_offset_ns, -42);
}

TEST(ClosedLoopScene, RejectsAMissingOrEmptyWindow)
{
  auto missing = scene_json();
  missing.erase("window");
  auto empty = scene_json();
  empty["window"]["end_ns"] = empty["window"]["start_ns"];
  auto old = scene_json();
  old["schema_version"] = 2;

  EXPECT_THROW(parse_closed_loop_scene(missing), std::invalid_argument);
  EXPECT_THROW(parse_closed_loop_scene(empty), std::invalid_argument);
  EXPECT_THROW(parse_closed_loop_scene(old), std::invalid_argument);
}

TEST(ClosedLoopEpisode, LaysTheEgoOnRowsFromTheWindowStart)
{
  const auto episode = build_closed_loop_episode(
    drive(99.0, 103.0, 5.0), {}, drive(99.0, 104.0, 5.0), at(100.0), at(102.0), at(102.0));

  EXPECT_EQ(episode.window_rows, 20U);
  ASSERT_EQ(episode.executed_rows(), 21U);
  EXPECT_EQ(episode.ego.start_time, at(100.0));
  EXPECT_EQ(episode.row_time(10), at(101.0));
  EXPECT_NEAR(episode.ego.points.at(10).pose.position.x, 10.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(episode.ego.points.at(10).speed_mps, 5.0);
  EXPECT_EQ(episode.ego.points.at(10).time_from_start, rclcpp::Duration::from_seconds(1.0));
  EXPECT_EQ(episode.recorded_poses.size(), 21U);
}

TEST(ClosedLoopEpisode, ARunThatEndedEarlyHasFewerExecutedRows)
{
  const auto episode = build_closed_loop_episode(
    drive(99.0, 101.0, 5.0), {}, drive(99.0, 104.0, 5.0), at(100.0), at(102.0), at(103.0));

  EXPECT_EQ(episode.window_rows, 20U);
  EXPECT_EQ(episode.executed_rows(), 11U);
  // The recorded drive is laid out to the later of the window's end and the run's end.
  EXPECT_EQ(episode.recorded_poses.size(), 31U);
}

TEST(ClosedLoopEpisode, NeedsOdometryAtTheWindowStart)
{
  EXPECT_THROW(
    build_closed_loop_episode(drive(100.5, 103.0, 5.0), {}, {}, at(100.0), at(102.0), at(102.0)),
    std::invalid_argument);
}

TEST(ClosedLoopEpisode, FitsRoadUserSpeedsFromPositionsOnRows)
{
  // Logged every 0.1 s halfway between rows, with a logged speed of 0 that must not be used.
  const auto objects = track(1, ObjectClassification::CAR, 99.5, 103.0, 20.0, 3.5, 2.0, 0.0);

  const auto episode = build_closed_loop_episode(
    drive(99.0, 103.0, 5.0), objects, {}, at(100.0), at(102.0), at(102.0));

  ASSERT_EQ(episode.road_users.size(), 1U);
  const auto & road_user = episode.road_users.front();
  ASSERT_EQ(road_user.states.size(), 21U);
  EXPECT_EQ(track_rows(episode, road_user).front(), 0U);
  EXPECT_NEAR(road_user.states.at(10).pose.position.x, 20.0 + 2.0 * 1.5, 1.0e-6);
  EXPECT_NEAR(road_user.states.at(10).twist.linear.x, 2.0, 1.0e-6);
  EXPECT_NEAR(road_user.states.front().twist.linear.x, 2.0, 1.0e-6);
}

TEST(ClosedLoopEpisode, KeepsRoadUsersAndDropsBlipsAndOtherObjects)
{
  auto objects = track(1, ObjectClassification::PEDESTRIAN, 99.5, 103.0, 10.0, 3.0, 1.0);
  objects = merge(objects, track(2, ObjectClassification::UNKNOWN, 99.5, 103.0, 10.0, -3.0, 0.0));
  // Seen for 0.5 s only: a detection blip.
  objects = merge(objects, track(3, ObjectClassification::CAR, 100.5, 101.0, 30.0, 0.0, 0.0));

  const auto episode = build_closed_loop_episode(
    drive(99.0, 103.0, 5.0), objects, {}, at(100.0), at(102.0), at(102.0));

  ASSERT_EQ(episode.road_users.size(), 1U);
  EXPECT_EQ(episode.road_users.front().object_id.uuid.front(), 1U);
  EXPECT_EQ(episode.dropped_short_tracks, 1U);
}

TEST(ClosedLoopEpisode, SplitsARoadUserAtAGapInItsTrack)
{
  auto objects = track(4, ObjectClassification::CAR, 99.5, 100.6, 20.0, 3.5, 0.0);
  objects = merge(objects, track(4, ObjectClassification::CAR, 101.4, 103.0, 20.0, 3.5, 0.0));

  const auto episode = build_closed_loop_episode(
    drive(99.0, 103.0, 5.0), objects, {}, at(100.0), at(102.0), at(102.0));

  ASSERT_EQ(episode.road_users.size(), 2U);
  EXPECT_EQ(episode.road_users.at(0).object_id, episode.road_users.at(1).object_id);
  EXPECT_EQ(track_rows(episode, episode.road_users.at(0)).back(), 5U);
  EXPECT_EQ(track_rows(episode, episode.road_users.at(1)).front(), 15U);
}

TEST(ObjectTracks, ALoggedStampReturnsItsOwnState)
{
  metrics::LoggedObjectTrack logged;
  logged.has_valid_object_id = true;
  logged.object_id.uuid = std::array<std::uint8_t, 16>{5};
  for (const auto & [time, speed] :
       std::vector<std::pair<double, double>>{{10.0, 1.0}, {11.0, 7.0}}) {
    metrics::LoggedObjectState state;
    state.stamp = at(time);
    state.pose = make_pose(time, 0.0);
    state.twist.linear.x = speed;
    state.shape.dimensions.x = 4.0;
    state.shape.dimensions.y = 1.8;
    logged.states.push_back(state);
  }

  const auto object = metrics::interpolate_logged_object_state(logged, at(11.0));

  ASSERT_TRUE(object.has_value());
  EXPECT_DOUBLE_EQ(object->speed_mps, 7.0);
}

TEST(ClosedLoopEvaluator, AnEgoThatDrivesIntoAStandingCarFails)
{
  // The car stands with its rear at 18 m. The ego is at 5 m at the window's start, so its front
  // bumper (3.74 m ahead of it) gets there after 1.852 s: row 19 is the first row past it.
  const auto objects = track(6, ObjectClassification::CAR, 99.5, 104.0, 20.0, 0.0, 0.0);
  const auto episode = build_closed_loop_episode(
    drive(99.0, 104.0, 5.0), objects, {}, at(100.0), at(103.0), at(103.0));
  const ClosedLoopEvaluator evaluator(nullptr, make_vehicle_info());

  const auto result =
    evaluator.evaluate(parse_closed_loop_scene(scene_json_without_label()), episode);

  ASSERT_TRUE(result.passed.has_value());
  EXPECT_FALSE(*result.passed);
  EXPECT_EQ(result.fail_reasons, std::vector<std::string>{"collision"});
  ASSERT_EQ(result.no_at_fault_collision.contacts.size(), 1U);
  EXPECT_EQ(result.no_at_fault_collision.contacts.front().point_index, 19U);
  EXPECT_EQ(result.no_at_fault_collision.contacts.front().collision_type, "STOPPED_TRACK");
  const auto json = to_json(parse_closed_loop_scene(scene_json()), episode, result);
  EXPECT_EQ(json.at("verdict").at("passed"), false);
  EXPECT_EQ(json.at("episode").at("window_rows"), 30U);
  EXPECT_EQ(json.at("no_at_fault_collision").at("contacts").size(), 1U);
}

TEST(ClosedLoopEvaluator, ACarThatRunsIntoTheEgoFromBehindDoesNotFailIt)
{
  // A faster car catching the ego up from behind, in its lane.
  const auto objects = track(7, ObjectClassification::CAR, 99.5, 104.0, -12.0, 0.0, 9.0);
  const auto episode = build_closed_loop_episode(
    drive(99.0, 104.0, 5.0), objects, {}, at(100.0), at(103.0), at(103.0));
  const ClosedLoopEvaluator evaluator(nullptr, make_vehicle_info());

  const auto result =
    evaluator.evaluate(parse_closed_loop_scene(scene_json_without_label()), episode);

  ASSERT_TRUE(result.passed.has_value());
  EXPECT_TRUE(*result.passed);
  ASSERT_EQ(result.no_at_fault_collision.contacts.size(), 1U);
  EXPECT_FALSE(result.no_at_fault_collision.contacts.front().at_fault);
}

TEST(ClosedLoopBagReader, FindsTheOffsetOfAReplayedDrive)
{
  // A simulation that replayed the recorded drive 37.25 s later in its clock, then stopped.
  const auto recorded = drive(99.0, 104.0, 5.0);
  auto executed = drive(99.0 + 37.25, 102.0 + 37.25, 5.0);

  const auto offset = estimate_time_offset(executed, recorded, at(99.5), at(101.5));

  ASSERT_TRUE(offset.has_value());
  EXPECT_NEAR(static_cast<double>(*offset) * 1.0e-9, 37.25, 1.0e-6);
  // A recorded ego that stood still there gives none.
  EXPECT_FALSE(
    estimate_time_offset(executed, drive(99.0, 104.0, 0.0), at(99.5), at(101.5)).has_value());
}

TEST(ClosedLoopScene, ReadsAWindowInTheRecordedClockWithAnAutomaticOffset)
{
  auto json = scene_json();
  json["window_clock"] = "recorded";
  json["reference"] = {
    {"bag_path", "/data/recorded"}, {"time_offset_ns", "auto"}, {"match_s", 4.0}};

  const auto scene = parse_closed_loop_scene(json);

  EXPECT_TRUE(scene.window_in_recorded_clock);
  EXPECT_TRUE(scene.reference_time_offset_auto);
  EXPECT_DOUBLE_EQ(scene.reference_match_s, 4.0);
  auto wrong = scene_json();
  wrong["window_clock"] = "sim";
  EXPECT_THROW(parse_closed_loop_scene(wrong), std::invalid_argument);
}

TEST(ClosedLoopBagReader, SelectsTheRouteInForceAtTheWindowStart)
{
  std::vector<autoware_planning_msgs::msg::LaneletRoute> routes(3);
  routes.at(0).header.stamp = at(10.0);
  routes.at(0).start_pose.position.x = 1.0;
  routes.at(1).header.stamp = at(50.0);
  routes.at(1).start_pose.position.x = 2.0;
  routes.at(2).header.stamp = at(90.0);
  routes.at(2).start_pose.position.x = 3.0;

  EXPECT_DOUBLE_EQ(select_route(routes, at(60.0))->start_pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(select_route(routes, at(5.0))->start_pose.position.x, 1.0);
  EXPECT_FALSE(select_route({}, at(5.0)).has_value());
}

}  // namespace autoware::planning_data_analyzer::closed_loop
