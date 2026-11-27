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

#include "closed_loop/contacts.hpp"

#include "closed_loop/road_users.hpp"
#include "metrics/geometry/ego_footprint.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

metrics::NoAtFaultCollisionResult episode_contacts(
  const ClosedLoopEpisode & episode, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double rear_heading_deg,
  const std::vector<bool> & straddling_excused)
{
  auto footprints = metrics::evaluate_ego_path_footprints(episode.ego, vehicle_info);
  const auto boxes = ego_boxes(episode, vehicle_info);
  for (std::size_t row = 0; row < footprints.size() && row < boxes.size(); ++row) {
    const auto flags = layout.flags(boxes.at(row));
    metrics::EgoAreaEvaluation area;
    area.flags.multiple_lanes = flags.straddles;
    area.flags.non_drivable_area = flags.off_road;
    footprints.at(row).ego_area_evaluation = area;
  }
  metrics::NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;
  options.from_behind_heading_deg = rear_heading_deg;
  options.straddling_excused = straddling_excused;
  return metrics::calculate_no_at_fault_collision(
    episode.ego, episode.road_users, vehicle_info, nullptr, footprints, false, options);
}

std::string contact_text(const metrics::NoAtFaultCollisionContact & contact)
{
  if (contact.exempt) {
    return "from behind";
  }
  if (contact.collision_type == "ACTIVE_LATERAL") {
    if (contact.multiple_lanes && !contact.straddling_excused) {
      return "side, straddling lanes";
    }
    if (contact.non_drivable_area) {
      return "side, off the road";
    }
    return contact.multiple_lanes ? "side, straddling lanes on the recorded path"
                                  : "side, in its lane";
  }
  if (contact.collision_type == "STOPPED_EGO") {
    return "ego stopped";
  }
  if (contact.collision_type == "STOPPED_TRACK") {
    return "road user stopped";
  }
  if (contact.collision_type == "ACTIVE_REAR") {
    return "from behind";
  }
  return "front";
}

ClosedLoopEpisode without_road_users(
  const ClosedLoopEpisode & episode, const std::vector<std::string> & ignored_track_ids)
{
  ClosedLoopEpisode kept = episode;
  if (ignored_track_ids.empty()) {
    return kept;
  }
  const std::set<std::string> ignored(ignored_track_ids.begin(), ignored_track_ids.end());
  kept.road_users.erase(
    std::remove_if(
      kept.road_users.begin(), kept.road_users.end(),
      [&](const auto & track) {
        return ignored.count(
                 metrics::object_id_to_string(track.object_id, track.has_valid_object_id)) != 0U;
      }),
    kept.road_users.end());
  return kept;
}

}  // namespace autoware::planning_data_analyzer::closed_loop
