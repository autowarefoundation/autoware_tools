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

#ifndef CLOSED_LOOP__CLOSED_LOOP_EPISODE_HPP_
#define CLOSED_LOOP__CLOSED_LOOP_EPISODE_HPP_

#include "data_types.hpp"
#include "metrics/geometry/ego_path.hpp"
#include "metrics/geometry/object_tracks.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// How an episode is laid out on rows. The defaults are those of the reviewed closed-loop metrics.
struct ClosedLoopEpisodeOptions
{
  double interval_s{0.1};
  // A road user seen on fewer rows than this is a detection blip and is dropped.
  double min_track_s{1.0};
  // A road user's speed on a row is the slope of a straight line fitted to its positions on the
  // rows within this either side (replayed tracks carry no reliable velocity).
  double speed_half_window_s{0.5};
  // Box sides below this are raised to it.
  double min_box_side_m{0.1};
  // A road user is seen on a row between two of its logged states at most this far apart.
  double max_track_gap_s{0.3};
};

// One executed closed-loop run on a uniform time grid. Row k is at ego.start_time + k * interval_s;
// row 0 is the scene window's start.
struct ClosedLoopEpisode
{
  double interval_s{0.1};
  // Rows of the scene window, from row 0: its length over the interval, rounded (a 10.0 s window
  // is 100 rows). More than the executed rows when the run ended early.
  std::size_t window_rows{0};
  // The executed ego on every executed row (pose and planar speed from its odometry).
  metrics::EgoPath ego;
  // Road users (pedestrians, bicycles and vehicles) as tracks with one state per row on which
  // they are seen, each state's twist.linear.x holding the fitted speed. A road user seen on
  // separate stretches of rows has one track per stretch, all with its object ID.
  std::vector<metrics::LoggedObjectTrack> road_users;
  // The recorded (human) drive on rows from row 0, as far as it reaches (at most to the later of
  // the window's end and the run's end).
  std::vector<geometry_msgs::msg::Pose> recorded_poses;
  std::size_t dropped_short_tracks{0};
  // The object IDs of the road users dropped as detection blips (seen for less than min_track_s).
  std::vector<std::string> short_track_ids;

  std::size_t executed_rows() const { return ego.points.size(); }
  rclcpp::Time row_time(std::size_t row) const;
};

// Lays a run out on rows from window_start. Odometry and objects are sorted by stamp; the recorded
// odometry is already in the executed clock. Executed rows end at run_end or where the executed
// odometry ends, whichever is first. Throws std::invalid_argument when the options are not
// positive, the window is empty, or no executed odometry covers the window's start.
ClosedLoopEpisode build_closed_loop_episode(
  const std::vector<Odometry> & executed_odometry,
  const std::vector<TimedTrackedObjects> & tracked_objects,
  const std::vector<Odometry> & recorded_odometry, const rclcpp::Time & window_start,
  const rclcpp::Time & window_end, const rclcpp::Time & run_end,
  const ClosedLoopEpisodeOptions & options = {});

// The rows of each road user track, in the order of its states.
std::vector<std::size_t> track_rows(
  const ClosedLoopEpisode & episode, const metrics::LoggedObjectTrack & track);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__CLOSED_LOOP_EPISODE_HPP_
