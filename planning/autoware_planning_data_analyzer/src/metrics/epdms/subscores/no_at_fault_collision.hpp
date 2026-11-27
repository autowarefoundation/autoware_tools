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

#ifndef METRICS__EPDMS__SUBSCORES__NO_AT_FAULT_COLLISION_HPP_
#define METRICS__EPDMS__SUBSCORES__NO_AT_FAULT_COLLISION_HPP_

#include "data_types.hpp"
#include "metrics/geometry/ego_footprint.hpp"
#include "metrics/geometry/ego_path.hpp"
#include "metrics/geometry/object_tracks.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

struct NoAtFaultCollisionDebugEvent
{
  double time_s{0.0};
  std::string object_id{"invalid"};
  std::string object_label{"UNKNOWN"};
  std::string collision_type{"NONE"};
  std::string reason{"available"};
  double event_score{1.0};
  bool agent{false};
  bool at_fault{false};
  bool ego_stopped{false};
  bool track_stopped{false};
  bool behind{false};
  bool front_hit{false};
  bool multiple_lanes{false};
  bool non_drivable_area{false};
  geometry_msgs::msg::Point ego_center;
  geometry_msgs::msg::Point object_center;
  std::vector<geometry_msgs::msg::Point> ego_footprint;
  std::vector<geometry_msgs::msg::Point> object_footprint;
  std::vector<geometry_msgs::msg::Point> front_bumper;
};

struct NoAtFaultCollisionHorizonFootprint
{
  double time_s{0.0};
  std::string object_id{"ego"};
  std::string object_label{"EGO"};
  bool collision{false};
  bool at_fault{false};
  std::vector<geometry_msgs::msg::Point> footprint;
};

struct NoAtFaultCollisionOverlapArea
{
  double time_s{0.0};
  std::string object_id{"invalid"};
  std::string object_label{"UNKNOWN"};
  bool at_fault{false};
  std::vector<geometry_msgs::msg::Point> polygon;
};

struct NoAtFaultCollisionDebugInfo
{
  std::vector<NoAtFaultCollisionDebugEvent> events;
  std::vector<NoAtFaultCollisionHorizonFootprint> ego_horizon_footprints;
  std::vector<NoAtFaultCollisionHorizonFootprint> object_horizon_footprints;
  std::vector<NoAtFaultCollisionOverlapArea> overlap_areas;
};

// How NC judges contacts. The defaults are the open-loop rule; closed-loop evaluation of a driven
// path sets the rules its reviewed metrics use.
struct NoAtFaultCollisionOptions
{
  // Judge each object at its first contact only, as NAVSIM does: a contact that is not the ego's
  // fault settles that object, and later contacts with it are not judged. Off: a later contact
  // with the same object is judged again. Objects without a valid ID are judged at every contact.
  bool first_contact_decides{false};
  // A contact with an object whose centre is behind the ego's rear bumper and that heads within
  // this many degrees of the ego is never the ego's fault: the object drove into the ego. Unset:
  // no such exemption.
  std::optional<double> from_behind_heading_deg;
  // One flag per ego path point (empty: none): straddling lanes on that point is not the ego's
  // fault, e.g. while the ego follows the recorded drive's path. Being off the drivable area
  // still is.
  std::vector<bool> straddling_excused;
};

// One object's first contact with the ego (NoAtFaultCollisionOptions::first_contact_decides).
struct NoAtFaultCollisionContact
{
  std::string object_id{"invalid"};
  std::string object_label{"UNKNOWN"};
  std::size_t point_index{0};
  double time_s{0.0};
  std::string collision_type{"NONE"};
  bool at_fault{false};
  // At fault by the rule, but excused by from_behind_heading_deg.
  bool exempt{false};
  bool multiple_lanes{false};
  bool non_drivable_area{false};
  bool straddling_excused{false};
};

struct NoAtFaultCollisionResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  double infraction_time_s{std::numeric_limits<double>::infinity()};
  // Each object's first contact, in the order they happen; filled when
  // NoAtFaultCollisionOptions::first_contact_decides is set.
  std::vector<NoAtFaultCollisionContact> contacts;
  NoAtFaultCollisionDebugInfo debug_info;
};

// NC of any timed ego path: a planned trajectory (to_ego_path) or the path a closed-loop ego drove.
// footprint_evaluations holds one evaluation per path point (evaluate_ego_path_footprints); logged
// objects are looked up at ego_path.start_time + each point's time_from_start.
NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const EgoPath & ego_path, const std::vector<LoggedObjectTrack> & object_tracks,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<TrajectoryFootprintEvaluation> & footprint_evaluations,
  bool collect_debug = false, const NoAtFaultCollisionOptions & options = {});

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<LoggedObjectTrack> & object_tracks,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<TrajectoryFootprintEvaluation> & footprint_evaluations,
  bool collect_debug = false);

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler = nullptr, bool collect_debug = false);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__NO_AT_FAULT_COLLISION_HPP_
