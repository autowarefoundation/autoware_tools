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

#include "object_tracks.hpp"

#include "metric_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

struct ObjectIdHash
{
  std::size_t operator()(const std::array<uint8_t, 16> & object_id) const
  {
    std::size_t seed = 0U;
    for (const auto byte : object_id) {
      seed ^= std::hash<uint8_t>{}(byte) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    }
    return seed;
  }
};

double closest_pi_symmetric_yaw(const double reference_yaw, const double yaw)
{
  double best_yaw = yaw;
  double best_error = std::abs(yaw - reference_yaw);
  for (int multiplier = -1; multiplier <= 1; ++multiplier) {
    const double candidate_yaw = yaw + static_cast<double>(multiplier) * M_PI;
    const double candidate_error = std::abs(candidate_yaw - reference_yaw);
    if (candidate_error < best_error) {
      best_yaw = candidate_yaw;
      best_error = candidate_error;
    }
  }
  return best_yaw;
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double planar_speed_mps(const geometry_msgs::msg::Twist & twist)
{
  return std::hypot(twist.linear.x, twist.linear.y);
}

double planar_displacement_m(
  const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
{
  return autoware_utils_geometry::calc_distance2d(lhs.position, rhs.position);
}

bool should_hold_long_object_yaw(
  const LoggedObjectState & previous_state, const LoggedObjectState & current_state,
  const double previous_yaw, const double current_yaw)
{
  // These thresholds suppress tracker yaw flips for long slow/stationary objects, where
  // a bounding box can jump by about pi without the object physically turning.
  constexpr double kLongObjectLengthThresholdM = 8.0;
  constexpr double kSlowObjectSpeedThresholdMps = 2.0;
  constexpr double kSmallDisplacementThresholdM = 0.75;
  constexpr double kLargeYawJumpThresholdRad = 10.0 * M_PI / 180.0;

  const double object_length =
    std::max(previous_state.shape.dimensions.x, current_state.shape.dimensions.x);
  if (object_length < kLongObjectLengthThresholdM) {
    return false;
  }

  const double max_speed =
    std::max(planar_speed_mps(previous_state.twist), planar_speed_mps(current_state.twist));
  if (max_speed > kSlowObjectSpeedThresholdMps) {
    return false;
  }

  const double displacement = planar_displacement_m(previous_state.pose, current_state.pose);
  if (displacement > kSmallDisplacementThresholdM) {
    return false;
  }

  const double yaw_delta = std::abs(normalize_angle(current_yaw - previous_yaw));
  return yaw_delta > kLargeYawJumpThresholdRad;
}

void canonicalize_bounding_box_yaws(LoggedObjectTrack & track)
{
  if (track.states.size() < 2U) {
    return;
  }

  bool has_reference_yaw = false;
  double reference_yaw = 0.0;
  const LoggedObjectState * previous_state = nullptr;
  for (auto & state : track.states) {
    if (state.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      has_reference_yaw = false;
      previous_state = nullptr;
      continue;
    }

    const double raw_yaw = get_yaw(state.pose.orientation);
    double canonical_yaw =
      has_reference_yaw ? closest_pi_symmetric_yaw(reference_yaw, raw_yaw) : raw_yaw;
    if (
      has_reference_yaw && previous_state &&
      should_hold_long_object_yaw(*previous_state, state, reference_yaw, canonical_yaw)) {
      canonical_yaw = reference_yaw;
    }
    state.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(canonical_yaw);
    reference_yaw = canonical_yaw;
    has_reference_yaw = true;
    previous_state = &state;
  }
}

}  // namespace

bool has_valid_object_id(const unique_identifier_msgs::msg::UUID & object_id)
{
  return std::any_of(
    object_id.uuid.begin(), object_id.uuid.end(), [](const auto byte) { return byte != 0U; });
}

std::array<uint8_t, 16> object_id_key(const unique_identifier_msgs::msg::UUID & object_id)
{
  return object_id.uuid;
}

bool is_agent_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  const auto label = autoware::object_recognition_utils::getHighestProbLabel(classification);
  return autoware::object_recognition_utils::isVehicle(label) ||
         label == ObjectClassification::PEDESTRIAN || label == ObjectClassification::ANIMAL;
}

bool is_unknown_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  return autoware::object_recognition_utils::getHighestProbLabel(classification) ==
         ObjectClassification::UNKNOWN;
}

std::vector<LoggedObjectTrack> build_logged_object_tracks(
  const std::vector<TimedTrackedObjects> & future_objects)
{
  std::unordered_map<std::array<uint8_t, 16>, LoggedObjectTrack, ObjectIdHash> keyed_tracks;
  std::vector<LoggedObjectTrack> invalid_id_tracks;

  for (const auto & timed_objects : future_objects) {
    if (!timed_objects.objects) {
      continue;
    }
    for (const auto & object : timed_objects.objects->objects) {
      LoggedObjectState state;
      state.stamp = timed_objects.stamp;
      state.pose = object.kinematics.pose_with_covariance.pose;
      state.twist = object.kinematics.twist_with_covariance.twist;
      state.shape = object.shape;
      state.classification = object.classification;

      const bool valid_id = has_valid_object_id(object.object_id);
      if (!valid_id) {
        LoggedObjectTrack track;
        track.has_valid_object_id = false;
        track.states.push_back(state);
        invalid_id_tracks.push_back(track);
        continue;
      }

      const auto key = object_id_key(object.object_id);
      auto & track = keyed_tracks[key];
      track.object_id = key;
      track.has_valid_object_id = true;
      track.states.push_back(state);
    }
  }

  std::vector<LoggedObjectTrack> tracks;
  tracks.reserve(keyed_tracks.size() + invalid_id_tracks.size());
  for (auto & [_, track] : keyed_tracks) {
    std::sort(track.states.begin(), track.states.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.stamp.nanoseconds() < rhs.stamp.nanoseconds();
    });
    canonicalize_bounding_box_yaws(track);
    tracks.push_back(std::move(track));
  }
  std::sort(tracks.begin(), tracks.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.object_id < rhs.object_id;
  });
  for (auto & track : invalid_id_tracks) {
    tracks.push_back(std::move(track));
  }
  return tracks;
}

std::vector<TimedTrackedObjects> get_future_objects_for_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & object_timeline, const double horizon_s)
{
  std::vector<TimedTrackedObjects> future_objects;
  if (trajectory.points.empty() || object_timeline.empty()) {
    return future_objects;
  }

  const auto trajectory_start_ns = rclcpp::Time(trajectory.header.stamp).nanoseconds();
  auto trajectory_horizon_ns =
    rclcpp::Duration(trajectory.points.back().time_from_start).nanoseconds();
  if (horizon_s > 0.0) {
    trajectory_horizon_ns =
      std::min(trajectory_horizon_ns, rclcpp::Duration::from_seconds(horizon_s).nanoseconds());
  }

  // This is not message sync tolerance. Keep one extra tracker sample beyond the trajectory
  // horizon so interpolation at the last trajectory point can still use a following object state.
  constexpr rcutils_time_point_value_t kFutureObjectRangeMarginNs =
    static_cast<rcutils_time_point_value_t>(200'000'000);
  const auto range_end_ns =
    trajectory_start_ns + trajectory_horizon_ns + kFutureObjectRangeMarginNs;

  const auto first = std::lower_bound(
    object_timeline.begin(), object_timeline.end(), trajectory_start_ns,
    [](const TimedTrackedObjects & timed_objects, const rcutils_time_point_value_t stamp_ns) {
      return timed_objects.stamp.nanoseconds() < stamp_ns;
    });

  for (auto itr = first; itr != object_timeline.end(); ++itr) {
    if (itr->stamp.nanoseconds() > range_end_ns) {
      break;
    }
    future_objects.push_back(*itr);
  }
  return future_objects;
}

std::optional<InterpolatedLoggedObject> interpolate_logged_object_state(
  const LoggedObjectTrack & track, const rclcpp::Time & query_time)
{
  if (track.states.empty()) {
    return std::nullopt;
  }

  auto make_object = [&](const LoggedObjectState & state, const double speed_mps) {
    InterpolatedLoggedObject object;
    object.object_id = track.object_id;
    object.has_valid_object_id = track.has_valid_object_id;
    object.pose = state.pose;
    object.speed_mps = speed_mps;
    object.shape = state.shape;
    object.classification = state.classification;
    object.polygon = autoware_utils_geometry::to_polygon2d(object.pose, object.shape);
    return object;
  };

  if (track.states.size() == 1U) {
    const auto & state = track.states.front();
    constexpr int64_t kSingleStateToleranceNs = 50'000'000;
    if (
      std::llabs(query_time.nanoseconds() - state.stamp.nanoseconds()) > kSingleStateToleranceNs) {
      return std::nullopt;
    }
    return make_object(state, std::hypot(state.twist.linear.x, state.twist.linear.y));
  }

  if (
    query_time.nanoseconds() < track.states.front().stamp.nanoseconds() ||
    query_time.nanoseconds() > track.states.back().stamp.nanoseconds()) {
    return std::nullopt;
  }

  auto after = std::lower_bound(
    track.states.begin(), track.states.end(), query_time,
    [](const auto & state, const auto & time) {
      return state.stamp.nanoseconds() < time.nanoseconds();
    });
  if (after == track.states.begin()) {
    return make_object(*after, std::hypot(after->twist.linear.x, after->twist.linear.y));
  }
  if (after == track.states.end()) {
    const auto & state = track.states.back();
    return make_object(state, std::hypot(state.twist.linear.x, state.twist.linear.y));
  }

  const auto & previous = *(after - 1);
  const auto & next = *after;
  const double dt = (next.stamp - previous.stamp).seconds();
  if (dt <= 0.0) {
    return make_object(next, std::hypot(next.twist.linear.x, next.twist.linear.y));
  }

  const double ratio = std::clamp((query_time - previous.stamp).seconds() / dt, 0.0, 1.0);
  InterpolatedLoggedObject object;
  object.object_id = track.object_id;
  object.has_valid_object_id = track.has_valid_object_id;
  object.pose =
    autoware_utils_geometry::calc_interpolated_pose(previous.pose, next.pose, ratio, false);
  const double logged_speed = std::hypot(previous.twist.linear.x, previous.twist.linear.y);
  if (std::isfinite(logged_speed) && logged_speed > 1.0e-6) {
    object.speed_mps = logged_speed;
  } else {
    object.speed_mps =
      autoware_utils_geometry::calc_distance2d(previous.pose.position, next.pose.position) / dt;
  }
  object.shape = previous.shape;
  object.classification = previous.classification;
  object.polygon = autoware_utils_geometry::to_polygon2d(object.pose, object.shape);
  return object;
}

}  // namespace autoware::planning_data_analyzer::metrics
