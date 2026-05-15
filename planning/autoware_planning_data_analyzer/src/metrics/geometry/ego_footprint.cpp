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

#include "ego_footprint.hpp"

#include "lanelet_geometry.hpp"
#include "lanelet_queries.hpp"
#include "metric_utils.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/utility/Utilities.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

constexpr double kSemanticDrivableAreaSearchMarginM = 15.0;
constexpr double kRoadBorderSearchMarginM = 5.0;
constexpr double kRoadBorderMaxGapM = 3.0;
constexpr double kRoadBorderMaxSemanticToBorderM = 4.0;
constexpr double kRoadBorderBetweenToleranceM = 0.75;
constexpr double kRoadBorderMaxTangentAlignment = 0.6;
constexpr double kMinRoadBorderSegmentLengthM = 1.0e-3;
constexpr double kRoadBorderSideEpsilon = 1.0e-3;
constexpr std::array<double, 8> kRoadBorderSideProbeDistancesM{0.3, 0.6, 1.0, 1.5,
                                                               2.0, 2.5, 3.0, 4.0};

bool footprint_intersects_lanelet(
  const autoware_utils_geometry::Polygon2d & footprint, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return !bg::disjoint(footprint, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

std::vector<autoware_utils_geometry::Point2d> footprint_vertices(
  const autoware_utils_geometry::Polygon2d & footprint)
{
  namespace bg = boost::geometry;

  std::vector<autoware_utils_geometry::Point2d> vertices;
  for (const auto & point : footprint.outer()) {
    if (vertices.empty() || !bg::equals(vertices.front(), point)) {
      vertices.push_back(point);
    }
  }
  return vertices;
}

lanelet::BoundingBox2d footprint_bounding_box(
  const autoware_utils_geometry::Polygon2d & footprint, const double search_margin_m = 1.0e-3)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & point : footprint.outer()) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }

  return make_bounding_box(min_x, min_y, max_x, max_y, search_margin_m);
}

lanelet::ConstLanelets collect_candidate_road_lanelets(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler,
  const lanelet::ConstLanelets & designated_lanelets)
{
  lanelet::ConstLanelets road_lanelets;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return road_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & lanelet : designated_lanelets) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    append_unique_lanelet(lanelet, road_lanelets, seen_ids);
  }

  const auto map = route_handler->getLaneletMapPtr();
  for (const auto & lanelet : map->laneletLayer.search(
         footprint_bounding_box(ego_polygon, kSemanticDrivableAreaSearchMarginM))) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    append_unique_lanelet(lanelet, road_lanelets, seen_ids);
  }

  return road_lanelets;
}

lanelet::ConstLanelets collect_candidate_shoulder_lanelets(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets shoulder_lanelets;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return shoulder_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  const auto map = route_handler->getLaneletMapPtr();
  for (const auto & lanelet : map->laneletLayer.search(
         footprint_bounding_box(ego_polygon, kSemanticDrivableAreaSearchMarginM))) {
    if (!route_handler->isShoulderLanelet(lanelet)) {
      continue;
    }
    append_unique_lanelet(lanelet, shoulder_lanelets, seen_ids);
  }

  return shoulder_lanelets;
}

std::vector<lanelet::ConstPolygon3d> collect_candidate_map_polygons(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::unordered_set<std::string> & types)
{
  std::vector<lanelet::ConstPolygon3d> polygons;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return polygons;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & polygon : map->polygonLayer.search(
         footprint_bounding_box(ego_polygon, kSemanticDrivableAreaSearchMarginM))) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (types.count(type) == 0U) {
      continue;
    }
    append_unique_polygon(polygon, polygons, seen_ids);
  }

  return polygons;
}

std::vector<lanelet::ConstLineString3d> collect_candidate_road_border_lines(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<lanelet::ConstLineString3d> road_border_lines;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return road_border_lines;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  const auto search_bbox = footprint_bounding_box(ego_polygon, kRoadBorderSearchMarginM);
  for (const auto & line_string : map->lineStringLayer.search(search_bbox)) {
    const std::string type = line_string.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "road_border") {
      continue;
    }
    append_unique_line_string(line_string, road_border_lines, seen_ids);
  }

  return road_border_lines;
}

bool point_in_semantic_drivable_area(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  for (const auto & lanelet : road_lanelets) {
    if (point_in_lanelet(point, lanelet)) {
      return true;
    }
  }
  for (const auto & lanelet : shoulder_lanelets) {
    if (point_in_lanelet(point, lanelet)) {
      return true;
    }
  }
  for (const auto & polygon : intersection_areas) {
    if (point_in_polygon(point, polygon)) {
      return true;
    }
  }
  for (const auto & polygon : hatched_road_markings) {
    if (point_in_polygon(point, polygon)) {
      return true;
    }
  }
  for (const auto & polygon : parking_lots) {
    if (point_in_polygon(point, polygon)) {
      return true;
    }
  }
  return false;
}

double distance_to_semantic_drivable_area(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  namespace bg = boost::geometry;

  double min_distance_m = std::numeric_limits<double>::infinity();
  const auto update_min_distance = [&point, &min_distance_m](const auto & polygon) {
    min_distance_m = std::min(min_distance_m, bg::distance(point, polygon));
  };

  for (const auto & lanelet : road_lanelets) {
    update_min_distance(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
  }
  for (const auto & lanelet : shoulder_lanelets) {
    update_min_distance(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
  }
  for (const auto & polygon : intersection_areas) {
    update_min_distance(to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
  }
  for (const auto & polygon : hatched_road_markings) {
    update_min_distance(to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
  }
  for (const auto & polygon : parking_lots) {
    update_min_distance(to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
  }
  return min_distance_m;
}

struct ClosestSemanticBoundaryPoint
{
  autoware_utils_geometry::Point2d point;
  double distance_m{std::numeric_limits<double>::infinity()};
};

geometry_msgs::msg::Point to_msg_point(const autoware_utils_geometry::Point2d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = 0.0;
  return msg;
}

autoware_utils_geometry::Point2d closest_point_on_segment(
  const autoware_utils_geometry::Point2d & point, const autoware_utils_geometry::Point2d & start,
  const autoware_utils_geometry::Point2d & end)
{
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  const double length_sq = dx * dx + dy * dy;
  if (length_sq < kMinRoadBorderSegmentLengthM * kMinRoadBorderSegmentLengthM) {
    return start;
  }
  const double projection =
    ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / length_sq;
  const double clamped_projection = std::clamp(projection, 0.0, 1.0);
  return autoware_utils_geometry::Point2d{
    start.x() + clamped_projection * dx, start.y() + clamped_projection * dy};
}

void update_closest_boundary_from_polygon(
  const autoware_utils_geometry::Point2d & point,
  const autoware_utils_geometry::Polygon2d & polygon,
  std::optional<ClosestSemanticBoundaryPoint> & best)
{
  const auto & ring = polygon.outer();
  if (ring.size() < 2U) {
    return;
  }
  for (std::size_t index = 1; index < ring.size(); ++index) {
    const auto closest = closest_point_on_segment(point, ring.at(index - 1U), ring.at(index));
    const double distance = std::sqrt(
      autoware_utils_geometry::calc_squared_distance2d(to_msg_point(point), to_msg_point(closest)));
    if (!best.has_value() || distance < best->distance_m) {
      best = ClosestSemanticBoundaryPoint{closest, distance};
    }
  }
}

std::optional<ClosestSemanticBoundaryPoint> find_closest_semantic_boundary_point(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  std::optional<ClosestSemanticBoundaryPoint> best;
  for (const auto & lanelet : road_lanelets) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet.polygon2d().basicPolygon()), best);
  }
  for (const auto & lanelet : shoulder_lanelets) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet.polygon2d().basicPolygon()), best);
  }
  for (const auto & polygon : intersection_areas) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), best);
  }
  for (const auto & polygon : hatched_road_markings) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), best);
  }
  for (const auto & polygon : parking_lots) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), best);
  }
  return best;
}

std::vector<bool> evaluate_road_border_side_fallback(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const std::vector<bool> & semantic_corner_drivable, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots,
  const std::vector<lanelet::ConstLineString3d> & road_border_lines,
  std::vector<RoadBorderFallbackProbe> & side_tests)
{
  std::vector<bool> corner_drivable_by_road_border(footprint_points.size(), false);
  for (std::size_t index = 0; index < footprint_points.size(); ++index) {
    if (semantic_corner_drivable.at(index)) {
      continue;
    }
    const auto semantic_boundary = find_closest_semantic_boundary_point(
      footprint_points.at(index), road_lanelets, shoulder_lanelets, intersection_areas,
      hatched_road_markings, parking_lots);
    if (!semantic_boundary.has_value()) {
      continue;
    }

    std::optional<RoadBorderFallbackProbe> best_rejected_test;
    std::optional<RoadBorderFallbackProbe> best_accepted_test;
    for (const auto & line_string : road_border_lines) {
      const auto line_2d = lanelet::utils::to2D(line_string);
      if (line_2d.size() < 2U) {
        continue;
      }
      for (std::size_t segment_index = 1; segment_index < line_2d.size(); ++segment_index) {
        RoadBorderFallbackProbe test;
        test.corner_index = index;
        test.corner = footprint_points.at(index);
        test.semantic_closest_point = semantic_boundary->point;
        test.segment_start = autoware_utils_geometry::Point2d{
          line_2d[segment_index - 1U].x(), line_2d[segment_index - 1U].y()};
        test.segment_end =
          autoware_utils_geometry::Point2d{line_2d[segment_index].x(), line_2d[segment_index].y()};
        const double segment_dx = test.segment_end.x() - test.segment_start.x();
        const double segment_dy = test.segment_end.y() - test.segment_start.y();
        const double segment_length = std::hypot(segment_dx, segment_dy);
        if (segment_length < kMinRoadBorderSegmentLengthM) {
          continue;
        }

        test.closest_point =
          closest_point_on_segment(test.corner, test.segment_start, test.segment_end);
        test.distance_m = std::sqrt(
          autoware_utils_geometry::calc_squared_distance2d(
            to_msg_point(test.corner), to_msg_point(test.closest_point)));
        test.corner_semantic_distance_m = semantic_boundary->distance_m;

        const double semantic_to_border_x = test.closest_point.x() - semantic_boundary->point.x();
        const double semantic_to_border_y = test.closest_point.y() - semantic_boundary->point.y();
        const double semantic_to_border_sq =
          semantic_to_border_x * semantic_to_border_x + semantic_to_border_y * semantic_to_border_y;
        test.semantic_to_border_distance_m = std::sqrt(semantic_to_border_sq);
        if (semantic_to_border_sq > kMinRoadBorderSegmentLengthM * kMinRoadBorderSegmentLengthM) {
          const double corner_vector_x = test.corner.x() - semantic_boundary->point.x();
          const double corner_vector_y = test.corner.y() - semantic_boundary->point.y();
          test.corner_between_ratio =
            (corner_vector_x * semantic_to_border_x + corner_vector_y * semantic_to_border_y) /
            semantic_to_border_sq;
          const auto projected_on_semantic_to_border = autoware_utils_geometry::Point2d{
            semantic_boundary->point.x() + test.corner_between_ratio * semantic_to_border_x,
            semantic_boundary->point.y() + test.corner_between_ratio * semantic_to_border_y};
          test.corner_to_semantic_border_line_m = std::sqrt(
            autoware_utils_geometry::calc_squared_distance2d(
              to_msg_point(test.corner), to_msg_point(projected_on_semantic_to_border)));
          const double tangent_x = segment_dx / segment_length;
          const double tangent_y = segment_dy / segment_length;
          test.border_tangent_alignment = std::abs(
            (semantic_to_border_x / test.semantic_to_border_distance_m) * tangent_x +
            (semantic_to_border_y / test.semantic_to_border_distance_m) * tangent_y);
        }

        const double normal_x = -segment_dy / segment_length;
        const double normal_y = segment_dx / segment_length;
        bool found_unambiguous_side = false;
        for (const double probe_distance_m : kRoadBorderSideProbeDistancesM) {
          test.plus_sample = autoware_utils_geometry::Point2d{
            test.closest_point.x() + normal_x * probe_distance_m,
            test.closest_point.y() + normal_y * probe_distance_m};
          test.minus_sample = autoware_utils_geometry::Point2d{
            test.closest_point.x() - normal_x * probe_distance_m,
            test.closest_point.y() - normal_y * probe_distance_m};
          test.plus_sample_drivable = point_in_semantic_drivable_area(
            test.plus_sample, road_lanelets, shoulder_lanelets, intersection_areas,
            hatched_road_markings, parking_lots);
          test.minus_sample_drivable = point_in_semantic_drivable_area(
            test.minus_sample, road_lanelets, shoulder_lanelets, intersection_areas,
            hatched_road_markings, parking_lots);
          test.plus_sample_semantic_distance_m =
            test.plus_sample_drivable ? 0.0
                                      : distance_to_semantic_drivable_area(
                                          test.plus_sample, road_lanelets, shoulder_lanelets,
                                          intersection_areas, hatched_road_markings, parking_lots);
          test.minus_sample_semantic_distance_m =
            test.minus_sample_drivable ? 0.0
                                       : distance_to_semantic_drivable_area(
                                           test.minus_sample, road_lanelets, shoulder_lanelets,
                                           intersection_areas, hatched_road_markings, parking_lots);
          if (test.plus_sample_drivable != test.minus_sample_drivable) {
            found_unambiguous_side = true;
            break;
          }
        }

        if (found_unambiguous_side) {
          const auto road_side_sample =
            test.plus_sample_drivable ? test.plus_sample : test.minus_sample;
          const double corner_side = (test.corner.x() - test.closest_point.x()) * normal_x +
                                     (test.corner.y() - test.closest_point.y()) * normal_y;
          const double road_side = (road_side_sample.x() - test.closest_point.x()) * normal_x +
                                   (road_side_sample.y() - test.closest_point.y()) * normal_y;
          const bool same_road_side = corner_side * road_side > kRoadBorderSideEpsilon;
          const bool bounded_gap =
            test.corner_between_ratio >= -0.05 && test.corner_between_ratio <= 1.05 &&
            test.corner_to_semantic_border_line_m <= kRoadBorderBetweenToleranceM;
          const bool across_border =
            test.border_tangent_alignment <= kRoadBorderMaxTangentAlignment;
          test.accepted = same_road_side && bounded_gap && across_border &&
                          test.distance_m <= kRoadBorderMaxGapM &&
                          test.semantic_to_border_distance_m <= kRoadBorderMaxSemanticToBorderM;
        }

        if (!best_rejected_test.has_value() || test.distance_m < best_rejected_test->distance_m) {
          best_rejected_test = test;
        }
        if (
          test.accepted &&
          (!best_accepted_test.has_value() || test.distance_m < best_accepted_test->distance_m)) {
          best_accepted_test = test;
        }
      }
    }

    if (best_accepted_test.has_value()) {
      corner_drivable_by_road_border.at(index) = true;
      side_tests.push_back(*best_accepted_test);
    } else if (best_rejected_test.has_value()) {
      side_tests.push_back(*best_rejected_test);
    }
  }
  return corner_drivable_by_road_border;
}

bool detect_multiple_lanes(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const lanelet::ConstLanelets & road_lanelets)
{
  if (footprint_points.empty() || road_lanelets.empty()) {
    return false;
  }

  std::size_t occupied_lanelets = 0;
  bool single_lane_contains_all_points = false;
  for (const auto & lanelet : road_lanelets) {
    std::size_t contained_points = 0;
    for (const auto & point : footprint_points) {
      if (point_in_lanelet(point, lanelet)) {
        ++contained_points;
      }
    }

    if (contained_points > 0U) {
      ++occupied_lanelets;
    }
    if (contained_points == footprint_points.size()) {
      single_lane_contains_all_points = true;
    }
  }

  return occupied_lanelets > 1U && !single_lane_contains_all_points;
}

std::vector<bool> evaluate_corner_drivable(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const lanelet::ConstLanelets & road_lanelets, const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  std::vector<bool> corner_drivable(footprint_points.size(), false);

  for (std::size_t index = 0; index < footprint_points.size(); ++index) {
    const auto & point = footprint_points.at(index);
    for (const auto & lanelet : road_lanelets) {
      if (point_in_lanelet(point, lanelet)) {
        corner_drivable.at(index) = true;
        break;
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & lanelet : shoulder_lanelets) {
        if (point_in_lanelet(point, lanelet)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & intersection_area : intersection_areas) {
        if (point_in_polygon(point, intersection_area)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & hatched_road_marking : hatched_road_markings) {
        if (point_in_polygon(point, hatched_road_marking)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & parking_lot : parking_lots) {
        if (point_in_polygon(point, parking_lot)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
  }

  return corner_drivable;
}

}  // namespace

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return create_pose_footprint(pose, vehicle_info.createFootprint(0.0));
}

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware_utils_geometry::LinearRing2d & local_footprint)
{
  autoware_utils_geometry::Polygon2d polygon;
  polygon.outer() = autoware_utils_geometry::transform_vector(
    local_footprint, autoware_utils_geometry::pose2transform(pose));
  boost::geometry::correct(polygon);
  return polygon;
}

std::optional<EgoAreaEvaluation> compute_ego_area_evaluation(
  const geometry_msgs::msg::Pose & pose, const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler,
  const lanelet::ConstLanelets & designated_lanelets, const bool evaluate_intersection_context)
{
  if (!route_handler) {
    return std::nullopt;
  }
  if (!route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  auto road_lanelets =
    collect_candidate_road_lanelets(ego_polygon, route_handler, designated_lanelets);
  auto shoulder_lanelets = collect_candidate_shoulder_lanelets(ego_polygon, route_handler);
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (
      route_handler->isRoadLanelet(lanelet) && footprint_intersects_lanelet(ego_polygon, lanelet)) {
      const auto duplicate = std::any_of(
        road_lanelets.begin(), road_lanelets.end(),
        [&lanelet](const auto & candidate) { return candidate.id() == lanelet.id(); });
      if (!duplicate) {
        road_lanelets.push_back(lanelet);
      }
    }
  }
  const auto intersection_areas =
    collect_candidate_map_polygons(ego_polygon, route_handler, {"intersection_area"});
  const auto hatched_road_markings =
    collect_candidate_map_polygons(ego_polygon, route_handler, {"hatched_road_markings"});
  const auto parking_lots =
    collect_candidate_map_polygons(ego_polygon, route_handler, {"parking_lot"});
  auto road_border_lines = collect_candidate_road_border_lines(ego_polygon, route_handler);
  const auto points = footprint_vertices(ego_polygon);
  const auto semantic_corner_drivable = evaluate_corner_drivable(
    points, road_lanelets, shoulder_lanelets, intersection_areas, hatched_road_markings,
    parking_lots);
  auto corner_drivable = semantic_corner_drivable;
  std::vector<RoadBorderFallbackProbe> road_border_side_tests;
  auto corner_drivable_by_road_border = evaluate_road_border_side_fallback(
    points, semantic_corner_drivable, road_lanelets, shoulder_lanelets, intersection_areas,
    hatched_road_markings, parking_lots, road_border_lines, road_border_side_tests);
  for (std::size_t index = 0; index < corner_drivable.size(); ++index) {
    if (!corner_drivable.at(index) && corner_drivable_by_road_border.at(index)) {
      corner_drivable.at(index) = true;
    }
  }

  EgoAreaEvaluation evaluation;
  evaluation.flags.multiple_lanes = detect_multiple_lanes(points, road_lanelets);
  evaluation.flags.non_drivable_area = std::any_of(
    corner_drivable.begin(), corner_drivable.end(), [](const bool value) { return !value; });
  if (evaluate_intersection_context) {
    const auto intersection_context = compute_driving_direction_local_context(pose, route_handler);
    evaluation.flags.intersection_context_available = intersection_context.has_value();
    evaluation.flags.in_intersection =
      intersection_context.has_value() && intersection_context->in_intersection;
  }
  evaluation.flags.road_border_fallback_used = false;
  for (std::size_t index = 0; index < corner_drivable.size(); ++index) {
    if (!semantic_corner_drivable.at(index) && corner_drivable_by_road_border.at(index)) {
      evaluation.flags.road_border_fallback_used = true;
      break;
    }
  }
  evaluation.footprint_points = points;
  evaluation.corner_drivable = corner_drivable;
  evaluation.corner_drivable_by_road_border = std::move(corner_drivable_by_road_border);
  evaluation.road_border_side_tests = std::move(road_border_side_tests);
  evaluation.road_lanelets = std::move(road_lanelets);
  evaluation.shoulder_lanelets = std::move(shoulder_lanelets);
  evaluation.intersection_areas = std::move(intersection_areas);
  evaluation.hatched_road_markings = std::move(hatched_road_markings);
  evaluation.parking_lots = std::move(parking_lots);
  evaluation.road_border_lines = std::move(road_border_lines);
  evaluation.designated_lanelet_count = designated_lanelets.size();
  return evaluation;
}

std::vector<TrajectoryFootprintEvaluation> evaluate_trajectory_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const lanelet::ConstLanelets * route_relevant_lanelets, const bool evaluate_intersection_context)
{
  std::vector<TrajectoryFootprintEvaluation> evaluations;
  if (!is_vehicle_info_valid(vehicle_info)) {
    return evaluations;
  }

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  if (local_footprint.empty()) {
    return evaluations;
  }

  const auto local_designated_lanelets =
    route_relevant_lanelets == nullptr && route_handler && route_handler->isHandlerReady()
      ? collect_route_relevant_lanelets(trajectory, route_handler)
      : lanelet::ConstLanelets{};
  const auto & designated_lanelets =
    route_relevant_lanelets != nullptr ? *route_relevant_lanelets : local_designated_lanelets;

  evaluations.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    TrajectoryFootprintEvaluation evaluation;
    evaluation.ego_polygon = create_pose_footprint(point.pose, local_footprint);
    if (route_handler) {
      evaluation.ego_area_evaluation = compute_ego_area_evaluation(
        point.pose, evaluation.ego_polygon, route_handler, designated_lanelets,
        evaluate_intersection_context);
    }
    evaluations.push_back(std::move(evaluation));
  }

  return evaluations;
}

}  // namespace autoware::planning_data_analyzer::metrics
