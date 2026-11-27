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

#ifndef CLOSED_LOOP__MARGIN_HPP_
#define CLOSED_LOOP__MARGIN_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/lane_layout.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// Why a road user's distance counts (line_m keys are the lower-case names).
enum class MarginRole {
  NOT_COUNTED = 0,
  ROAD_USER = 1,
  AVOIDED_OBJECT = 2,
  ONCOMING = 3,
  ONCOMING_VRU = 4
};

std::string to_string(MarginRole role);

// The thresholds of the margin verdict (the reviewed metric's values).
struct MarginParameters
{
  // The smallest distance to a road user of each role; every line is 0 m for now, so only a
  // collision the ego causes fails.
  std::map<std::string, double> line_m{
    {"road_user", 0.0}, {"avoided_object", 0.0}, {"oncoming", 0.0}, {"oncoming_vru", 0.0}};
  // The ego's slowest speed at which a distance counts.
  double moving_mps{0.5};
  // How close a vehicle, or a pedestrian or bicycle, comes to the executed or recorded ego to be
  // a candidate.
  double candidate_vehicle_m{15.0};
  double candidate_vru_m{5.0};
  // Within this of the recorded path, straddling lanes is not the ego's fault.
  double on_path_m{0.5};
  // The judging stops at a contact that is not the ego's fault when the road user drives more than
  // cut_depth_m into the ego within cut_window_s from that contact's row on.
  double cut_depth_m{0.5};
  double cut_window_s{3.0};
  // The lane the ego starts in: its lanelets within this of the recorded drive, widened.
  double start_lane_reach_m{4.0};
  double lane_tolerance_m{0.1};
  // Avoidance roles.
  double standing_mps{1.0};
  double oncoming_deg{120.0};
  double oncoming_range_m{40.0};
  double vru_moving_mps{1.0};
  // NC's from-behind exemption and the speed fit.
  double rear_heading_deg{45.0};
  double speed_half_window_s{0.5};
};

// What the runner knows of a scene for its margin (MarginReference).
struct MarginReference
{
  // lane_change, avoidance or pull_out.
  std::string kind;
  // Avoidance only: a scene with a trailing vehicle, in which every other road user counts as in a
  // lane change.
  bool trailing_vehicle{false};
  // An easy scene: every candidate road user counts as a road user on every row.
  bool every_road_user{false};
  // Road users never judged (e.g. known tracking errors).
  std::vector<std::string> ignored_track_ids;
};

struct MarginRecord
{
  std::string track_id;
  std::string type;
  std::string role;
  double distance_m{0.0};
  std::size_t row{0};
  double line_m{0.0};
  bool passed{true};
};

struct MarginContactRecord
{
  std::string track_id;
  std::string type;
  bool counted{false};
  std::size_t row{0};
  std::string kind;
  bool at_fault{false};
  bool judged{true};
};

struct MarginResult
{
  bool passed{true};
  std::vector<MarginRecord> road_users;
  std::vector<MarginContactRecord> contacts;
  std::vector<std::string> fail_reasons;
  // The row the judging stops at (a deep not-at-fault contact); none: the run's end.
  std::optional<std::size_t> judged_until_row;
  bool start_lane{false};
  std::size_t at_fault_collision_count{0};
  std::size_t margin_failure_count{0};
  std::size_t counted_road_user_count{0};
  std::optional<double> min_distance_m;
};

// Throws std::invalid_argument for an unknown kind, or a trailing vehicle outside an avoidance.
MarginResult evaluate_margin(
  const ClosedLoopEpisode & episode, const MarginReference & reference, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const MarginParameters & parameters = {});

nlohmann::json to_json(const MarginResult & result, const MarginReference & reference);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__MARGIN_HPP_
