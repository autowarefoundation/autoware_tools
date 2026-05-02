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

#include "traffic_light_compliance.hpp"

#include "../../geometry/metric_utils.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/utility/Utilities.h>

#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

using autoware_utils_geometry::LineString2d;

const autoware_perception_msgs::msg::TrafficLightGroup * find_signal_group(
  const TrafficLightGroupArray & traffic_signals, const lanelet::Id reg_elem_id)
{
  const auto it = std::find_if(
    traffic_signals.traffic_light_groups.begin(), traffic_signals.traffic_light_groups.end(),
    [reg_elem_id](const auto & group) {
      return static_cast<lanelet::Id>(group.traffic_light_group_id) == reg_elem_id;
    });
  return it == traffic_signals.traffic_light_groups.end() ? nullptr : &(*it);
}

bool footprint_intersects_stop_line(
  const geometry_msgs::msg::Pose & pose,
  const autoware_utils_geometry::LinearRing2d & local_footprint,
  const lanelet::ConstLineString3d & stop_line)
{
  const auto footprint_polygon = create_pose_footprint(pose, local_footprint);
  const auto stop_line_2d = to_linestring2d(stop_line);
  return boost::geometry::intersects(footprint_polygon, stop_line_2d);
}

}  // namespace

TrafficLightComplianceResult calculate_traffic_light_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<TrafficLightGroupArray> & traffic_signals,
  const std::shared_ptr<RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  TrafficLightComplianceResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (!route_handler) {
    result.reason = "unavailable_no_route_handler";
    return result;
  }
  if (!route_handler->isHandlerReady()) {
    result.reason = "unavailable_route_handler_not_ready";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  std::unordered_set<lanelet::Id> missing_signal_ids;
  bool encountered_relevant_traffic_light = false;
  result.available = true;
  result.reason = "available_no_relevant_traffic_lights";
  result.score = 1.0;
  const auto local_footprint = vehicle_info.createFootprint(0.0);

  for (const auto & point : trajectory.points) {
    const auto reference_lanelet = find_reference_lanelet(point.pose, route_handler);
    if (!reference_lanelet.has_value()) {
      continue;
    }

    for (const auto & reg_elem :
         reference_lanelet->regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      const auto stop_line = reg_elem->stopLine();
      if (!stop_line || stop_line->empty()) {
        continue;
      }

      encountered_relevant_traffic_light = true;
      if (!traffic_signals) {
        result.available = false;
        result.reason = "unavailable_no_traffic_signals";
        return result;
      }

      const auto * signal_group = find_signal_group(*traffic_signals, reg_elem->id());
      if (!signal_group) {
        missing_signal_ids.insert(reg_elem->id());
        continue;
      }

      if (!autoware::traffic_light_utils::isTrafficSignalStop(
            reference_lanelet.value(), *signal_group)) {
        result.reason = "available";
        continue;
      }

      if (footprint_intersects_stop_line(point.pose, local_footprint, *stop_line)) {
        result.reason = "red_light_stop_line_crossed";
        result.score = 0.0;
        return result;
      }

      result.reason = "available";
    }
  }

  if (!encountered_relevant_traffic_light) {
    return result;
  }

  if (!missing_signal_ids.empty()) {
    result.available = false;
    result.score = 0.0;
    result.reason = "unavailable_missing_signal_group";
    return result;
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
