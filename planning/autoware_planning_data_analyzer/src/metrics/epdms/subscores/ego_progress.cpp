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

#include "ego_progress.hpp"

#include "metrics/geometry/lanelet_queries.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

std::optional<double> calculate_raw_progress_m(
  const Trajectory & trajectory, const std::shared_ptr<RouteHandler> & route_handler,
  std::vector<geometry_msgs::msg::Point> * route_reference_points,
  const lanelet::ConstLanelets * route_relevant_lanelets)
{
  if (trajectory.points.size() < 2U) {
    return std::nullopt;
  }

  const auto local_route_lanelets = route_relevant_lanelets
                                      ? lanelet::ConstLanelets{}
                                      : collect_route_relevant_lanelets(trajectory, route_handler);
  const auto & route_lanelets =
    route_relevant_lanelets ? *route_relevant_lanelets : local_route_lanelets;
  if (route_lanelets.empty()) {
    return std::nullopt;
  }
  const auto lanelet_map_ptr = route_handler->getLaneletMapPtr();
  if (!lanelet_map_ptr) {
    return std::nullopt;
  }

  if (route_reference_points) {
    route_reference_points->clear();
    for (const auto & lanelet : route_lanelets) {
      for (const auto & point : lanelet.centerline()) {
        geometry_msgs::msg::Point msg_point;
        msg_point.x = point.x();
        msg_point.y = point.y();
        msg_point.z = point.z();
        route_reference_points->push_back(msg_point);
      }
    }
  }

  const auto start_arc =
    autoware::experimental::lanelet2_utils::get_arc_coordinates_on_ego_centerline(
      route_lanelets, trajectory.points.front().pose, lanelet_map_ptr);
  const auto end_arc =
    autoware::experimental::lanelet2_utils::get_arc_coordinates_on_ego_centerline(
      route_lanelets, trajectory.points.back().pose, lanelet_map_ptr);

  return std::max(end_arc.length - start_arc.length, 0.0);
}

}  // namespace

EgoProgressResult calculate_ego_progress(
  const std::shared_ptr<Trajectory> & selected_trajectory,
  const std::shared_ptr<RouteHandler> & route_handler, const double no_at_fault_collision,
  const bool no_at_fault_collision_available, const double drivable_area_compliance,
  const bool drivable_area_compliance_available, const double driving_direction_compliance,
  const bool driving_direction_compliance_available, const double traffic_light_compliance,
  const bool traffic_light_compliance_available,
  const lanelet::ConstLanelets * route_relevant_lanelets)
{
  EgoProgressResult result;

  if (!selected_trajectory) {
    result.reason = "unavailable_no_selected_trajectory";
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
  if (selected_trajectory->points.size() < 2U) {
    result.reason = "unavailable_short_trajectory";
    return result;
  }
  if (
    !no_at_fault_collision_available || !drivable_area_compliance_available ||
    !driving_direction_compliance_available || !traffic_light_compliance_available) {
    result.reason = "unavailable_missing_multiplicative_metric";
    return result;
  }

  result.start_point = selected_trajectory->points.front().pose.position;
  result.end_point = selected_trajectory->points.back().pose.position;
  const auto selected_raw_progress = calculate_raw_progress_m(
    *selected_trajectory, route_handler, &result.route_reference_points, route_relevant_lanelets);
  if (!selected_raw_progress.has_value()) {
    result.reason = "unavailable_no_route_lanelets";
    return result;
  }
  result.raw_progress_m = selected_raw_progress.value();
  result.multiplicative_mask = no_at_fault_collision * drivable_area_compliance *
                               driving_direction_compliance * traffic_light_compliance;
  result.denominator_m = result.raw_progress_m * result.multiplicative_mask;
  result.best_raw_progress_m = result.denominator_m;
  result.available = true;

  constexpr double kProgressDistanceThresholdM = 5.0;
  if (result.denominator_m > kProgressDistanceThresholdM) {
    result.score = std::clamp(result.raw_progress_m / result.denominator_m, 0.0, 1.0);
    result.reason = "available_single_proposal_navsim_ratio";
  } else {
    result.score = 1.0;
    result.reason = "available_single_proposal_navsim_fallback";
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
