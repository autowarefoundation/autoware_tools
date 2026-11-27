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

#ifndef CLOSED_LOOP__CONTACTS_HPP_
#define CLOSED_LOOP__CONTACTS_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/lane_layout.hpp"
#include "metrics/epdms/subscores/no_at_fault_collision.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// Each road user's first contact with the ego in one episode and whose fault it is: NC of the
// driven path with the closed-loop metrics' contact rule. Each road user is judged at its first
// contact; one centred behind the ego's rear bumper and heading within rear_heading_deg of it
// drove into it; straddling lanes is read on the lanes of the LaneLayout (a lanelet with its
// chains), and is no fault on rows of straddling_excused (one flag per row, or empty); being off
// the road is.
metrics::NoAtFaultCollisionResult episode_contacts(
  const ClosedLoopEpisode & episode, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, double rear_heading_deg,
  const std::vector<bool> & straddling_excused = {});

// A first contact in words, as the reviewed metrics report it.
std::string contact_text(const metrics::NoAtFaultCollisionContact & contact);

// The episode without the road users in ignored_track_ids.
ClosedLoopEpisode without_road_users(
  const ClosedLoopEpisode & episode, const std::vector<std::string> & ignored_track_ids);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__CONTACTS_HPP_
