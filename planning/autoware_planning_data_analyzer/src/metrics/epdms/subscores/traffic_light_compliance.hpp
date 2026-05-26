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

#ifndef METRICS__EPDMS__SUBSCORES__TRAFFIC_LIGHT_COMPLIANCE_HPP_
#define METRICS__EPDMS__SUBSCORES__TRAFFIC_LIGHT_COMPLIANCE_HPP_

#include "data_types.hpp"
#include "metrics/geometry/ego_footprint.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

struct TrafficLightComplianceDebugPolygon
{
  double time_s{0.0};
  std::vector<geometry_msgs::msg::Point> polygon;
  lanelet::Id source_id{0};
};

struct TrafficLightComplianceDebugInfo
{
  double first_failure_time_s{std::numeric_limits<double>::infinity()};
  geometry_msgs::msg::Point label_anchor;
  std::vector<TrafficLightComplianceDebugPolygon> ego_horizon_footprints;
  std::vector<TrafficLightComplianceDebugPolygon> stop_lines;
  std::vector<lanelet::Id> regulatory_element_ids;
  std::vector<lanelet::Id> selected_lane_ids;
  std::vector<lanelet::Id> stop_line_ids;
  std::size_t selected_stop_line_count{0};
  std::optional<std::string> intended_movement;
};

struct TrafficLightComplianceResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  TrafficLightComplianceDebugInfo debug_info;
};

TrafficLightComplianceResult calculate_traffic_light_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<TrafficLightGroupArray> & traffic_signals,
  const std::shared_ptr<RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<TurnIndicatorsReport> & turn_indicators_status = nullptr,
  const std::vector<TrajectoryFootprintEvaluation> * evaluations = nullptr,
  const lanelet::ConstLanelets * route_relevant_lanelets = nullptr);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__TRAFFIC_LIGHT_COMPLIANCE_HPP_
