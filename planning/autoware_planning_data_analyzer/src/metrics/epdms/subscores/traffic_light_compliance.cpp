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

#include "metrics/geometry/lanelet_queries.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

using TurnDirection = autoware::experimental::lanelet2_utils::TurnDirection;

std::vector<geometry_msgs::msg::Point> line_points(
  const lanelet::ConstLineString3d & line, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(line.size());
  for (const auto & point : line) {
    points.push_back(
      autoware_utils_geometry::to_msg(autoware_utils_geometry::Point3d{point.x(), point.y(), z}));
  }
  return points;
}

std::string turn_direction_to_string(const std::optional<TurnDirection> & turn_direction)
{
  if (!turn_direction.has_value()) {
    return "unknown";
  }
  switch (*turn_direction) {
    case TurnDirection::Straight:
      return "straight";
    case TurnDirection::Left:
      return "left";
    case TurnDirection::Right:
      return "right";
  }
  return "unknown";
}

struct RelevantTrafficLightGroup
{
  lanelet::Id regulatory_element_id{0};
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets selected_lanelets;
  std::optional<lanelet::ConstLineString3d> stop_line;
  std::optional<TurnDirection> intended_turn_direction;
};

const autoware_perception_msgs::msg::TrafficLightGroup * find_signal_group(
  const TrafficLightGroupArray & traffic_signals, const RelevantTrafficLightGroup & group)
{
  const auto it = std::find_if(
    traffic_signals.traffic_light_groups.begin(), traffic_signals.traffic_light_groups.end(),
    [regulatory_element_id = group.regulatory_element_id](const auto & signal_group) {
      return static_cast<lanelet::Id>(signal_group.traffic_light_group_id) == regulatory_element_id;
    });
  return it == traffic_signals.traffic_light_groups.end() ? nullptr : &(*it);
}

std::optional<TurnDirection> infer_turn_direction_from_indicator(
  const std::shared_ptr<TurnIndicatorsReport> & turn_indicators_status)
{
  if (!turn_indicators_status) {
    return std::nullopt;
  }

  switch (turn_indicators_status->report) {
    case TurnIndicatorsReport::ENABLE_LEFT:
      return TurnDirection::Left;
    case TurnIndicatorsReport::ENABLE_RIGHT:
      return TurnDirection::Right;
    default:
      return std::nullopt;
  }
}

std::optional<TurnDirection> infer_turn_direction_from_route_lanelets(
  const lanelet::ConstLanelets & lanelets)
{
  std::optional<TurnDirection> inferred;
  for (const auto & lanelet : lanelets) {
    const auto lanelet_turn_direction =
      autoware::experimental::lanelet2_utils::get_turn_direction(lanelet);
    if (!lanelet_turn_direction.has_value()) {
      continue;
    }
    if (!inferred.has_value()) {
      inferred = lanelet_turn_direction;
      continue;
    }
    if (inferred.value() != lanelet_turn_direction.value()) {
      return std::nullopt;
    }
  }
  return inferred;
}

bool matches_intended_turn_direction(
  const lanelet::ConstLanelet & lanelet,
  const std::optional<TurnDirection> & intended_turn_direction)
{
  if (!intended_turn_direction.has_value()) {
    return true;
  }

  const auto lanelet_turn_direction =
    autoware::experimental::lanelet2_utils::get_turn_direction(lanelet);
  if (!lanelet_turn_direction.has_value()) {
    return false;
  }
  return lanelet_turn_direction.value() == intended_turn_direction.value();
}

std::vector<RelevantTrafficLightGroup> build_relevant_traffic_light_groups(
  const lanelet::ConstLanelets & route_lanelets,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::shared_ptr<TurnIndicatorsReport> & turn_indicators_status)
{
  std::vector<RelevantTrafficLightGroup> groups;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return groups;
  }

  const auto indicator_turn_direction = infer_turn_direction_from_indicator(turn_indicators_status);
  std::unordered_map<lanelet::Id, std::size_t> index_by_reg_elem_id;
  for (const auto & lanelet : route_lanelets) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    for (const auto & reg_elem :
         lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      // TrafficLightGroup messages are keyed by regulatory element ID.
      const auto [it, inserted] =
        index_by_reg_elem_id.emplace(reg_elem->id(), index_by_reg_elem_id.size());
      if (inserted) {
        RelevantTrafficLightGroup group;
        group.regulatory_element_id = reg_elem->id();
        if (const auto stop_line = reg_elem->stopLine(); stop_line && !stop_line->empty()) {
          group.stop_line = *stop_line;
        }
        groups.push_back(group);
      }

      auto & group = groups[it->second];
      const auto duplicate = std::any_of(
        group.route_lanelets.begin(), group.route_lanelets.end(),
        [&lanelet](const auto & candidate) { return candidate.id() == lanelet.id(); });
      if (!duplicate) {
        group.route_lanelets.push_back(lanelet);
      }
      if (!group.stop_line.has_value()) {
        if (const auto stop_line = reg_elem->stopLine(); stop_line && !stop_line->empty()) {
          group.stop_line = *stop_line;
        }
      }
    }
  }

  for (auto & group : groups) {
    // Indicator state is sampled once at the trajectory timestamp; per-group intent inference can
    // be added later for multi-intersection horizons if needed.
    group.intended_turn_direction =
      indicator_turn_direction.has_value()
        ? indicator_turn_direction
        : infer_turn_direction_from_route_lanelets(group.route_lanelets);

    std::unordered_set<lanelet::Id> selected_lanelet_ids;
    for (const auto & lanelet : group.route_lanelets) {
      if (!matches_intended_turn_direction(lanelet, group.intended_turn_direction)) {
        continue;
      }
      if (selected_lanelet_ids.insert(lanelet.id()).second) {
        group.selected_lanelets.push_back(lanelet);
      }
    }

    if (group.selected_lanelets.empty()) {
      group.selected_lanelets = group.route_lanelets;
    }
  }

  return groups;
}

bool stop_line_intersects_ego_polygon(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const lanelet::ConstLineString3d & stop_line)
{
  return boost::geometry::intersects(ego_polygon, to_linestring2d(stop_line));
}

}  // namespace

TrafficLightComplianceResult calculate_traffic_light_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<TrafficLightGroupArray> & traffic_signals,
  const std::shared_ptr<RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<TurnIndicatorsReport> & turn_indicators_status,
  const std::vector<TrajectoryFootprintEvaluation> * evaluations,
  const lanelet::ConstLanelets * route_relevant_lanelets, const bool collect_debug)
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

  const auto local_route_lanelets = route_relevant_lanelets
                                      ? lanelet::ConstLanelets{}
                                      : collect_route_relevant_lanelets(trajectory, route_handler);
  const auto & route_lanelets =
    route_relevant_lanelets ? *route_relevant_lanelets : local_route_lanelets;
  const auto groups =
    build_relevant_traffic_light_groups(route_lanelets, route_handler, turn_indicators_status);
  for (const auto & group : groups) {
    if (collect_debug) {
      result.debug_info.regulatory_element_ids.push_back(group.regulatory_element_id);
      result.debug_info.intended_movement = turn_direction_to_string(group.intended_turn_direction);
      for (const auto & lanelet : group.selected_lanelets) {
        result.debug_info.selected_lane_ids.push_back(lanelet.id());
      }
      if (group.stop_line.has_value()) {
        result.debug_info.stop_line_ids.push_back(group.stop_line->id());
        result.debug_info.stop_lines.push_back(
          TrafficLightComplianceDebugPolygon{
            0.0, line_points(*group.stop_line, trajectory.points.front().pose.position.z),
            group.stop_line->id()});
      }
    }
  }
  result.debug_info.selected_stop_line_count =
    collect_debug ? result.debug_info.stop_lines.size() : 0U;
  result.available = true;
  result.reason = groups.empty() ? "available_no_relevant_traffic_lights" : "available";
  result.score = 1.0;
  if (groups.empty()) {
    return result;
  }

  if (!traffic_signals) {
    result.available = false;
    result.score = 0.0;
    result.reason = "unavailable_no_traffic_signals";
    return result;
  }

  std::unordered_set<lanelet::Id> missing_signal_ids_at_relevant_stop_line;
  const auto local_footprint = vehicle_info.createFootprint(0.0);
  const bool can_reuse_evaluations =
    evaluations != nullptr && evaluations->size() == trajectory.points.size();
  bool violation_found = false;

  for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points.at(point_index);
    const auto ego_polygon = can_reuse_evaluations
                               ? evaluations->at(point_index).ego_polygon
                               : create_pose_footprint(point.pose, local_footprint);
    if (collect_debug) {
      result.debug_info.ego_horizon_footprints.push_back(
        TrafficLightComplianceDebugPolygon{
          rclcpp::Duration(point.time_from_start).seconds(),
          polygon_to_points(ego_polygon, point.pose.position.z), 0});
    }

    for (const auto & group : groups) {
      if (!group.stop_line.has_value()) {
        continue;
      }
      if (!stop_line_intersects_ego_polygon(ego_polygon, *group.stop_line)) {
        continue;
      }

      const auto * signal_group = find_signal_group(*traffic_signals, group);
      if (!signal_group) {
        missing_signal_ids_at_relevant_stop_line.insert(group.regulatory_element_id);
        continue;
      }

      const bool stop_required = std::any_of(
        group.selected_lanelets.begin(), group.selected_lanelets.end(),
        [signal_group](const auto & lanelet) {
          return autoware::traffic_light_utils::isTrafficSignalStop(lanelet, *signal_group);
        });
      if (!stop_required) {
        continue;
      }

      if (!violation_found) {
        violation_found = true;
        result.reason = "red_light_stop_line_crossed";
        result.score = 0.0;
        if (collect_debug) {
          result.debug_info.first_failure_time_s =
            rclcpp::Duration(point.time_from_start).seconds();
          result.debug_info.label_anchor = point.pose.position;
        }
      }
    }
  }

  if (violation_found) {
    return result;
  }

  if (!missing_signal_ids_at_relevant_stop_line.empty()) {
    result.available = false;
    result.score = 0.0;
    result.reason = "unavailable_missing_signal_group";
    return result;
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
