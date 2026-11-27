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

#ifndef CLOSED_LOOP__LANE_GOAL_HPP_
#define CLOSED_LOOP__LANE_GOAL_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/lane_layout.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// The settings of the lane goal (the reviewed metric's values). The wheel tread comes from the
// vehicle info.
struct LaneGoalParameters
{
  double start_lane_reach_m{4.0};
  double lane_tolerance_m{0.1};
  // How long a run must stay in its goal lane (at least one row).
  double hold_s{0.0};
  // The recorded rows that decide a goal lane: the window's last for a lane change, its first for
  // an avoidance; a lane change's target lane holds the recorded wheels on more than this share.
  double goal_lane_s{1.0};
  double goal_lane_share{0.5};
  // A recorded drive already in the target lane all through this much of the window's start: the
  // lane change is over before the window (no verdict).
  double recorded_start_s{4.0};
  int successor_lanelets{2};
  double lane_heading_deg{45.0};
  // On its way in when the window ends: this share of the box in the goal lane on the window's last
  // row, up at least end_rise over the end_rise_s before.
  double end_share{0.6};
  double end_rise{0.05};
  double end_rise_s{1.0};
  // Avoidance: the avoided vehicle stands (median fitted speed below standing_mps) and comes within
  // avoided_reach_m of the recorded drive's box.
  double standing_mps{1.0};
  double avoided_reach_m{3.0};
  double speed_half_window_s{0.5};
};

// rules.GoalVerdict: reached (goal_row), failed (failure) or without a verdict (undecided).
struct GoalVerdict
{
  std::optional<std::size_t> goal_row;
  std::optional<std::size_t> recorded_goal_row;
  std::string failure;
  std::string undecided;
  std::optional<double> past_m;
  std::optional<std::size_t> progress_row;
  std::optional<std::size_t> recorded_progress_row;
  std::optional<double> max_progress_m;
  std::optional<double> end_share;
  std::optional<double> end_rise;
  bool moving_in{false};
  bool recorded_moving_in{false};

  std::optional<bool> passed() const
  {
    if (!undecided.empty()) {
      return std::nullopt;
    }
    return failure.empty();
  }
};

// rules.EndRule.
struct EndRule
{
  double share{0.6};
  double rise{0.05};
  std::size_t rise_rows{10};
  // The share on the window's last row and its rise over rise_rows rows before; none when the
  // shares do not reach them.
  std::optional<std::pair<double, double>> measure(
    const std::vector<double> & shares, std::size_t window) const;
  bool holds(const std::optional<std::pair<double, double>> & measured) const;
};

// rules.first_hold: the first row r in [start, stop) with inside[r, r + hold_rows) all true.
std::optional<std::size_t> first_hold(
  const std::vector<char> & inside, std::size_t start, std::size_t stop, std::size_t hold_rows);

// rules.choose_lane: the candidate holding the most rows, when more than share of them.
std::optional<std::size_t> choose_lane(const std::vector<std::vector<char>> & held, double share);

GoalVerdict lane_change_goal(
  const std::vector<char> & executed_inside, const std::vector<char> & recorded_inside,
  std::size_t window, std::size_t hold_rows, std::size_t start_rows, const EndRule & end,
  const std::vector<double> & executed_share, const std::vector<double> & recorded_share);

GoalVerdict avoidance_goal(
  const std::vector<char> & executed_inside, const std::vector<char> & recorded_inside,
  const std::vector<double> & executed_progress, const std::vector<double> & recorded_progress,
  double past_m, std::size_t window, std::size_t hold_rows, const EndRule & end,
  const std::vector<double> & executed_share, const std::vector<double> & recorded_share);

struct AvoidedVehicle
{
  std::string track_id;
  double far_m{0.0};
  double near_m{0.0};
  std::size_t candidates{0};
};

struct LaneGoalResult
{
  std::string kind;
  GoalVerdict verdict;
  bool lane_found{false};
  std::string lane_from;
  std::size_t lane_held_rows{0};
  std::vector<std::pair<std::size_t, std::size_t>> lane_runs;
  std::optional<AvoidedVehicle> avoided;
  std::vector<std::string> fail_reasons;
  std::string undecided_reason;
};

// Whether an easy lane change or avoidance ends in its lane inside the window. kind: lane_change or
// avoidance. Throws std::invalid_argument for another kind, a run without a recorded drive that
// covers the window, or a wheel tread wider than the ego.
LaneGoalResult evaluate_lane_goal(
  const ClosedLoopEpisode & episode, const std::string & kind,
  const std::vector<std::string> & ignored_track_ids, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const LaneGoalParameters & parameters = {});

nlohmann::json to_json(const LaneGoalResult & result);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__LANE_GOAL_HPP_
