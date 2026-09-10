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

#ifndef METRICS__EPDMS__SUBSCORES__TTC_WITHIN_BOUND_HPP_
#define METRICS__EPDMS__SUBSCORES__TTC_WITHIN_BOUND_HPP_

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

struct TTCWithinBoundDebugEvent
{
  double time_s{0.0};
  double future_offset_s{0.0};
  double query_time_s{0.0};
  std::string object_id{"invalid"};
  std::string object_label{"UNKNOWN"};
  bool ahead{false};
  bool behind{false};
  bool multiple_lanes{false};
  bool non_drivable_area{false};
  bool intersection{false};
  bool bad_or_intersection{false};
  bool ego_stopped{false};
  geometry_msgs::msg::Point ego_center;
  geometry_msgs::msg::Point object_center;
  std::vector<geometry_msgs::msg::Point> ego_footprint;
  std::vector<geometry_msgs::msg::Point> object_footprint;
};

struct TTCWithinBoundOverlapArea
{
  double time_s{0.0};
  double future_offset_s{0.0};
  std::string object_id{"invalid"};
  std::string object_label{"UNKNOWN"};
  bool failing{false};
  std::vector<geometry_msgs::msg::Point> polygon;
};

struct TTCWithinBoundHorizonFootprint
{
  double time_s{0.0};
  double future_offset_s{0.0};
  std::string object_id{"ego"};
  std::string object_label{"EGO"};
  bool prefix{false};
  bool overlap{false};
  bool failing{false};
  std::vector<geometry_msgs::msg::Point> footprint;
};

struct TTCWithinBoundDebugInfo
{
  std::vector<TTCWithinBoundDebugEvent> events;
  std::vector<TTCWithinBoundHorizonFootprint> ego_horizon_footprints;
  std::vector<TTCWithinBoundHorizonFootprint> object_horizon_footprints;
  std::vector<TTCWithinBoundOverlapArea> overlap_areas;
};

struct TTCWithinBoundResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  double infraction_time_s{std::numeric_limits<double>::infinity()};
  TTCWithinBoundDebugInfo debug_info;
};

TTCWithinBoundResult calculate_ttc_within_bound(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler = nullptr,
  const std::vector<TrajectoryFootprintEvaluation> * footprint_evaluations = nullptr,
  const std::vector<LoggedObjectTrack> * object_tracks = nullptr, bool collect_debug = false);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__TTC_WITHIN_BOUND_HPP_
