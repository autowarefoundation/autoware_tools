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
#include "metrics/geometry/object_tracks.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <limits>
#include <memory>
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

struct NoAtFaultCollisionResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  double infraction_time_s{std::numeric_limits<double>::infinity()};
  NoAtFaultCollisionDebugInfo debug_info;
};

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
