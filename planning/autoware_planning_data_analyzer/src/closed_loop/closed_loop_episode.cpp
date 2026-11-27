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

#include "metrics/geometry/metric_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

using autoware_perception_msgs::msg::ObjectClassification;

std::int64_t stamp_ns(const Odometry & odometry)
{
  return rclcpp::Time(odometry.header.stamp).nanoseconds();
}

double planar_speed(const Odometry & odometry)
{
  return std::hypot(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y);
}

struct PoseAndSpeed
{
  geometry_msgs::msg::Pose pose;
  double speed_mps{0.0};
};

// The odometry at time_ns, interpolated between the two samples around it; none outside them.
std::optional<PoseAndSpeed> sample_odometry(
  const std::vector<Odometry> & odometry, const std::int64_t time_ns)
{
  if (
    odometry.empty() || time_ns < stamp_ns(odometry.front()) ||
    time_ns > stamp_ns(odometry.back())) {
    return std::nullopt;
  }
  const auto after = std::lower_bound(
    odometry.begin(), odometry.end(), time_ns,
    [](const Odometry & sample, const std::int64_t time) { return stamp_ns(sample) < time; });
  if (after == odometry.begin() || stamp_ns(*after) == time_ns) {
    return PoseAndSpeed{after->pose.pose, planar_speed(*after)};
  }
  const auto & previous = *(after - 1);
  const double ratio = static_cast<double>(time_ns - stamp_ns(previous)) /
                       static_cast<double>(stamp_ns(*after) - stamp_ns(previous));
  PoseAndSpeed sample;
  sample.pose = autoware_utils_geometry::calc_interpolated_pose(
    previous.pose.pose, after->pose.pose, ratio, false);
  sample.speed_mps =
    planar_speed(previous) + ratio * (planar_speed(*after) - planar_speed(previous));
  return sample;
}

bool is_road_user_label(const std::uint8_t label)
{
  switch (label) {
    case ObjectClassification::CAR:
    case ObjectClassification::TRUCK:
    case ObjectClassification::BUS:
    case ObjectClassification::TRAILER:
    case ObjectClassification::MOTORCYCLE:
    case ObjectClassification::BICYCLE:
    case ObjectClassification::PEDESTRIAN:
      return true;
    default:
      return false;
  }
}

// The label a track carries on most of its states.
std::uint8_t track_label(const metrics::LoggedObjectTrack & track)
{
  std::map<std::uint8_t, std::size_t> counts;
  for (const auto & state : track.states) {
    ++counts[autoware::object_recognition_utils::getHighestProbLabel(state.classification)];
  }
  return std::max_element(
           counts.begin(), counts.end(),
           [](const auto & lhs, const auto & rhs) { return lhs.second < rhs.second; })
    ->first;
}

// The track's state at time_ns: a logged state at that time, or one interpolated between two
// logged states at most max_gap_ns apart.
std::optional<metrics::LoggedObjectState> sample_track(
  const metrics::LoggedObjectTrack & track, const std::int64_t time_ns,
  const std::int64_t max_gap_ns)
{
  const auto & states = track.states;
  if (
    states.empty() || time_ns < states.front().stamp.nanoseconds() ||
    time_ns > states.back().stamp.nanoseconds()) {
    return std::nullopt;
  }
  const auto after = std::lower_bound(
    states.begin(), states.end(), time_ns,
    [](const metrics::LoggedObjectState & state, const std::int64_t time) {
      return state.stamp.nanoseconds() < time;
    });
  if (after->stamp.nanoseconds() == time_ns) {
    return *after;
  }
  const auto & previous = *(after - 1);
  const auto gap_ns = after->stamp.nanoseconds() - previous.stamp.nanoseconds();
  if (gap_ns > max_gap_ns) {
    return std::nullopt;
  }
  const double ratio =
    static_cast<double>(time_ns - previous.stamp.nanoseconds()) / static_cast<double>(gap_ns);
  auto state = previous;
  state.pose =
    autoware_utils_geometry::calc_interpolated_pose(previous.pose, after->pose, ratio, false);
  return state;
}

// The speed on each row is the slope of the straight line fitted to the positions on the valid
// rows within +-half_window rows; 0 with fewer than two.
std::vector<double> fitted_speeds(
  const std::vector<std::optional<metrics::LoggedObjectState>> & rows,
  const std::size_t half_window, const double interval_s)
{
  std::vector<double> speeds(rows.size(), 0.0);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    if (!rows.at(row).has_value()) {
      continue;
    }
    const std::size_t low = row >= half_window ? row - half_window : 0U;
    const std::size_t high = std::min(rows.size(), row + half_window + 1U);
    double count = 0.0;
    double sum_t = 0.0;
    double sum_tt = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_tx = 0.0;
    double sum_ty = 0.0;
    for (std::size_t other = low; other < high; ++other) {
      if (!rows.at(other).has_value()) {
        continue;
      }
      const double tau = static_cast<double>(other);
      const auto & position = rows.at(other)->pose.position;
      count += 1.0;
      sum_t += tau;
      sum_tt += tau * tau;
      sum_x += position.x;
      sum_y += position.y;
      sum_tx += tau * position.x;
      sum_ty += tau * position.y;
    }
    const double spread = sum_tt - sum_t * sum_t / std::max(count, 1.0);
    if (count < 2.0 || spread <= 1.0e-12) {
      continue;
    }
    const double slope_x = (sum_tx - sum_t * sum_x / count) / spread;
    const double slope_y = (sum_ty - sum_t * sum_y / count) / spread;
    speeds.at(row) = std::hypot(slope_x, slope_y) / interval_s;
  }
  return speeds;
}

void raise_box_sides(autoware_perception_msgs::msg::Shape & shape, const double min_side_m)
{
  shape.dimensions.x = std::max(shape.dimensions.x, min_side_m);
  shape.dimensions.y = std::max(shape.dimensions.y, min_side_m);
}

}  // namespace

rclcpp::Time ClosedLoopEpisode::row_time(const std::size_t row) const
{
  const auto interval_ns = static_cast<std::int64_t>(std::llround(interval_s * 1.0e9));
  return ego.start_time +
         rclcpp::Duration::from_nanoseconds(interval_ns * static_cast<std::int64_t>(row));
}

ClosedLoopEpisode build_closed_loop_episode(
  const std::vector<Odometry> & executed_odometry,
  const std::vector<TimedTrackedObjects> & tracked_objects,
  const std::vector<Odometry> & recorded_odometry, const rclcpp::Time & window_start,
  const rclcpp::Time & window_end, const rclcpp::Time & run_end,
  const ClosedLoopEpisodeOptions & options)
{
  if (
    !(options.interval_s > 0.0) || !(options.speed_half_window_s > 0.0) ||
    !(options.max_track_gap_s > 0.0) || options.min_track_s < 0.0 || options.min_box_side_m < 0.0) {
    throw std::invalid_argument("closed-loop episode options must be positive");
  }
  if (window_end <= window_start) {
    throw std::invalid_argument("closed-loop episode: the window is empty");
  }
  const auto interval_ns = static_cast<std::int64_t>(std::llround(options.interval_s * 1.0e9));
  const auto start_ns = window_start.nanoseconds();
  const auto row_ns = [&](const std::size_t row) {
    return start_ns + interval_ns * static_cast<std::int64_t>(row);
  };

  ClosedLoopEpisode episode;
  episode.interval_s = options.interval_s;
  // As the reviewed metrics' scenes: the window's length in rows (10.0 s at 0.1 s is 100 rows).
  episode.window_rows = static_cast<std::size_t>(std::max<std::int64_t>(
    1, std::llround(
         static_cast<double>(window_end.nanoseconds() - start_ns) /
         static_cast<double>(interval_ns))));
  episode.ego.start_time = window_start;

  for (std::size_t row = 0; row_ns(row) <= run_end.nanoseconds(); ++row) {
    const auto sample = sample_odometry(executed_odometry, row_ns(row));
    if (!sample.has_value()) {
      break;
    }
    metrics::EgoPathPoint point;
    point.time_from_start = rclcpp::Duration::from_nanoseconds(row_ns(row) - start_ns);
    point.pose = sample->pose;
    point.speed_mps = sample->speed_mps;
    episode.ego.points.push_back(point);
  }
  if (episode.ego.points.empty()) {
    throw std::invalid_argument("closed-loop episode: no executed odometry at the window's start");
  }

  const auto recorded_end_ns = std::max(window_end.nanoseconds(), run_end.nanoseconds());
  for (std::size_t row = 0; row_ns(row) <= recorded_end_ns; ++row) {
    const auto sample = sample_odometry(recorded_odometry, row_ns(row));
    if (!sample.has_value()) {
      break;
    }
    episode.recorded_poses.push_back(sample->pose);
  }

  const auto rows = episode.executed_rows();
  const auto max_gap_ns = static_cast<std::int64_t>(std::llround(options.max_track_gap_s * 1.0e9));
  const auto min_rows =
    static_cast<std::size_t>(std::llround(options.min_track_s / options.interval_s));
  const auto half_window =
    static_cast<std::size_t>(std::llround(options.speed_half_window_s / options.interval_s));
  if (half_window < 1U) {
    throw std::invalid_argument("closed-loop episode: the speed fit must cover a row either side");
  }
  for (const auto & track : metrics::build_logged_object_tracks(tracked_objects)) {
    if (track.states.empty() || !is_road_user_label(track_label(track))) {
      continue;
    }
    std::vector<std::optional<metrics::LoggedObjectState>> samples(rows);
    std::size_t seen = 0;
    for (std::size_t row = 0; row < rows; ++row) {
      samples.at(row) = sample_track(track, row_ns(row), max_gap_ns);
      seen += samples.at(row).has_value() ? 1U : 0U;
    }
    if (seen == 0U) {
      continue;
    }
    if (seen < min_rows) {
      ++episode.dropped_short_tracks;
      episode.short_track_ids.push_back(
        metrics::object_id_to_string(track.object_id, track.has_valid_object_id));
      continue;
    }
    const auto speeds = fitted_speeds(samples, half_window, options.interval_s);
    metrics::LoggedObjectTrack stretch;
    const auto flush = [&]() {
      if (!stretch.states.empty()) {
        episode.road_users.push_back(std::move(stretch));
      }
      stretch = metrics::LoggedObjectTrack{};
    };
    for (std::size_t row = 0; row < rows; ++row) {
      if (!samples.at(row).has_value()) {
        flush();
        continue;
      }
      if (stretch.states.empty()) {
        stretch.object_id = track.object_id;
        stretch.has_valid_object_id = track.has_valid_object_id;
      }
      auto state = *samples.at(row);
      state.stamp = rclcpp::Time(row_ns(row), RCL_ROS_TIME);
      state.twist = geometry_msgs::msg::Twist{};
      state.twist.linear.x = speeds.at(row);
      raise_box_sides(state.shape, options.min_box_side_m);
      stretch.states.push_back(std::move(state));
    }
    flush();
  }
  return episode;
}

std::vector<std::size_t> track_rows(
  const ClosedLoopEpisode & episode, const metrics::LoggedObjectTrack & track)
{
  std::vector<std::size_t> rows;
  rows.reserve(track.states.size());
  const auto start_ns = episode.ego.start_time.nanoseconds();
  const double interval_ns = episode.interval_s * 1.0e9;
  for (const auto & state : track.states) {
    rows.push_back(
      static_cast<std::size_t>(
        std::llround(static_cast<double>(state.stamp.nanoseconds() - start_ns) / interval_ns)));
  }
  return rows;
}

}  // namespace autoware::planning_data_analyzer::closed_loop
