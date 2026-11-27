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

#ifndef CLOSED_LOOP__YIELD_TARGETS_HPP_
#define CLOSED_LOOP__YIELD_TARGETS_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/conflict.hpp"
#include "closed_loop/lane_layout.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// The thresholds of the target rules (the reviewed values). Distances in metres, speeds in m/s,
// angles in degrees, times in seconds.
struct TargetRules
{
  double near_m{3.0};
  int min_kept_rows{3};
  double oncoming_range_m{60.0};
  double oncoming_heading_deg{100.0};
  double oncoming_min_mps{1.0};
  double crosswalk_tolerance_m{0.5};
  double vru_near_m{3.0};
  double near_car_m{5.3};
  double approach_s{1.0};
  double near_pedestrian_m{0.5};
  double after_pet_s{3.0};
  double side_car_m{3.0};
  double same_direction_window_s{1.0};
  double same_direction_min_mps{1.0};
  double same_direction_lead_min_mps{0.3};
  double same_direction_deg{45.0};
  double velocity_half_window_s{0.5};
  double min_track_s{1.0};
  CrossingGrid grid;
};

struct TargetPick
{
  // In the order the road users first appear.
  std::vector<std::string> target_track_ids;
  // The rules that picked each target.
  std::map<std::string, std::vector<std::string>> reasons;
  // Rule 1's pick (the scene's own road user), when there is one.
  std::optional<std::string> final_track_id;
};

// Picks a yielding scene's targets from its recorded (human) drive and its road users: the
// episode's road users against the recorded ego (episode.recorded_poses). kind: vehicle_yield or
// pedestrian_yield. A road user is a target when a rule picks it (see README.md): 1 final, 2 ov,
// 3 vru, 4 near_car, 5 near_pedestrian, 6 right_after, 7 side_car, 8 same_direction. Broken tracks
// (one road user whose tracker ID changed) are not joined here: pass the targets in the scene file
// where that matters. Throws std::invalid_argument for another kind or without a recorded drive.
TargetPick pick_targets(
  const ClosedLoopEpisode & episode, const std::string & kind,
  const lanelet::LaneletMapConstPtr & map,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::vector<std::string> & ignored_track_ids, const TargetRules & rules = {});

nlohmann::json to_json(const TargetPick & pick);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__YIELD_TARGETS_HPP_
