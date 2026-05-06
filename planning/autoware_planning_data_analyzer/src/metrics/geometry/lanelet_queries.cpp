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

#include "lanelet_queries.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/utility/Utilities.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

constexpr double kLocalLaneSearchRadiusM = 5.0;
constexpr double kDirectionSimilarityThresholdRad = M_PI_4;
constexpr double kAdmissibleLaneMarginM = 0.35;

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(lanelet.id()).second) {
    lanelets.push_back(lanelet);
  }
}

void append_unique_polygon(
  const lanelet::ConstPolygon3d & polygon, std::vector<lanelet::ConstPolygon3d> & polygons,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(polygon.id()).second) {
    polygons.push_back(polygon);
  }
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon)
{
  namespace bg = boost::geometry;

  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : polygon) {
    converted.outer().push_back({point.x(), point.y()});
  }
  bg::correct(converted);
  return converted;
}

lanelet::BoundingBox2d point_bounding_box(
  const geometry_msgs::msg::Point & point, const double radius_m = kLocalLaneSearchRadiusM)
{
  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{point.x - radius_m, point.y - radius_m},
    lanelet::BasicPoint2d{point.x + radius_m, point.y + radius_m}};
}

bool point_in_lanelet(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

bool point_in_polygon(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstPolygon3d & polygon)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
}

bool point_within_lanelet_margin(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet,
  const double margin_m)
{
  namespace bg = boost::geometry;
  const auto polygon = to_polygon_2d(lanelet.polygon2d().basicPolygon());
  return bg::distance(point, polygon) <= margin_m;
}

lanelet::ConstLanelets collect_local_route_consistent_lanelets(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets local_lanelets;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return local_lanelets;
  }

  const auto map = route_handler->getLaneletMapPtr();
  const auto nearby_bbox = point_bounding_box(pose.position);
  std::unordered_set<lanelet::Id> nearby_road_lanelet_ids;
  std::unordered_set<lanelet::Id> nearby_shoulder_lanelet_ids;
  lanelet::ConstLanelets nearby_road_lanelets;
  lanelet::ConstLanelets nearby_shoulder_lanelets;
  for (const auto & lanelet : map->laneletLayer.search(nearby_bbox)) {
    if (route_handler->isRoadLanelet(lanelet)) {
      append_unique_lanelet(lanelet, nearby_road_lanelets, nearby_road_lanelet_ids);
    } else if (route_handler->isShoulderLanelet(lanelet)) {
      append_unique_lanelet(lanelet, nearby_shoulder_lanelets, nearby_shoulder_lanelet_ids);
    }
  }

  lanelet::ConstLanelets seed_lanelets;
  std::unordered_set<lanelet::Id> seed_ids;
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      append_unique_lanelet(lanelet, seed_lanelets, seed_ids);
    }
  }
  lanelet::ConstLanelet closest_route_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_route_lanelet)) {
    append_unique_lanelet(closest_route_lanelet, seed_lanelets, seed_ids);
  }

  if (seed_lanelets.empty()) {
    return local_lanelets;
  }

  const double reference_yaw =
    lanelet::utils::getLaneletAngle(seed_lanelets.front(), pose.position);
  std::unordered_set<lanelet::Id> local_lanelet_ids;
  for (const auto & lanelet : nearby_road_lanelets) {
    const bool on_route = route_handler->isRouteLanelet(lanelet);
    const double lanelet_yaw = lanelet::utils::getLaneletAngle(lanelet, pose.position);
    const bool same_direction =
      std::abs(normalize_angle(lanelet_yaw - reference_yaw)) <= kDirectionSimilarityThresholdRad;
    if (!on_route && !same_direction) {
      continue;
    }
    append_unique_lanelet(lanelet, local_lanelets, local_lanelet_ids);
  }

  for (const auto & shoulder_lanelet : nearby_shoulder_lanelets) {
    const double shoulder_yaw = lanelet::utils::getLaneletAngle(shoulder_lanelet, pose.position);
    const bool same_direction =
      std::abs(normalize_angle(shoulder_yaw - reference_yaw)) <= kDirectionSimilarityThresholdRad;
    if (!same_direction) {
      continue;
    }
    append_unique_lanelet(shoulder_lanelet, local_lanelets, local_lanelet_ids);
  }

  return local_lanelets;
}

std::vector<lanelet::ConstPolygon3d> collect_local_intersection_areas(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<lanelet::ConstPolygon3d> intersection_areas;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return intersection_areas;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & polygon : map->polygonLayer.search(point_bounding_box(pose.position))) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "intersection_area") {
      continue;
    }
    append_unique_polygon(polygon, intersection_areas, seen_ids);
  }

  return intersection_areas;
}

}  // namespace

std::optional<lanelet::ConstLanelet> find_reference_lanelet(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isHandlerReady()) {
    return std::nullopt;
  }

  lanelet::ConstLanelet closest_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_lanelet)) {
    return closest_lanelet;
  }

  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      return lanelet;
    }
  }

  return std::nullopt;
}

lanelet::ConstLanelets collect_route_relevant_lanelets(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets route_lanelets;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return route_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & point : trajectory.points) {
    for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(point.pose)) {
      if (!route_handler->isRouteLanelet(lanelet)) {
        continue;
      }
      append_unique_lanelet(lanelet, route_lanelets, seen_ids);
    }
  }

  return route_lanelets;
}

autoware_utils_geometry::LineString2d to_linestring2d(const lanelet::ConstLineString3d & line)
{
  autoware_utils_geometry::LineString2d line_2d;
  for (const auto & point : lanelet::utils::to2D(line)) {
    line_2d.push_back({point.x(), point.y()});
  }
  return line_2d;
}

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  const auto context = compute_driving_direction_local_context(pose, route_handler);
  return context.has_value() && context->in_intersection;
}

bool is_pose_in_route_lane_polygon(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  const auto context = compute_driving_direction_local_context(pose, route_handler);
  return context.has_value() && context->in_route_lane_polygon;
}

std::optional<DrivingDirectionLocalContext> compute_driving_direction_local_context(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  const autoware_utils_geometry::Point2d search_point{pose.position.x, pose.position.y};
  DrivingDirectionLocalContext context;
  context.route_lanelets = collect_local_route_consistent_lanelets(pose, route_handler);
  context.intersection_areas = collect_local_intersection_areas(pose, route_handler);

  const bool in_route_lane_polygon_exact = std::any_of(
    context.route_lanelets.begin(), context.route_lanelets.end(),
    [&search_point](const auto & lanelet) { return point_in_lanelet(search_point, lanelet); });
  const bool in_route_lane_polygon_margin = std::any_of(
    context.route_lanelets.begin(), context.route_lanelets.end(),
    [&search_point](const auto & lanelet) {
      return point_within_lanelet_margin(search_point, lanelet, kAdmissibleLaneMarginM);
    });
  context.in_route_lane_polygon = in_route_lane_polygon_exact || in_route_lane_polygon_margin;
  context.in_lane_margin_only = !in_route_lane_polygon_exact && in_route_lane_polygon_margin;
  context.in_intersection = std::any_of(
    context.intersection_areas.begin(), context.intersection_areas.end(),
    [&search_point](const auto & polygon) { return point_in_polygon(search_point, polygon); });

  if (!context.in_intersection) {
    for (const auto & lanelet : context.route_lanelets) {
      if (
        autoware::experimental::lanelet2_utils::is_intersection_lanelet(lanelet) &&
        point_in_lanelet(search_point, lanelet)) {
        context.in_intersection = true;
        break;
      }
    }
  }
  return context;
}

}  // namespace autoware::planning_data_analyzer::metrics
