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

#include "closed_loop/road_users.hpp"

#include "metrics/geometry/metric_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

RoadUserType type_of(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & labels)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (autoware::object_recognition_utils::getHighestProbLabel(labels)) {
    case ObjectClassification::PEDESTRIAN:
      return RoadUserType::PEDESTRIAN;
    case ObjectClassification::BICYCLE:
      return RoadUserType::BICYCLE;
    default:
      return RoadUserType::VEHICLE;
  }
}

// The distance from a point to a segment; past an end, to that end point.
double point_segment_distance(const Point2d & point, const Point2d & start, const Point2d & end)
{
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  const double length_sq = dx * dx + dy * dy;
  const double along =
    length_sq > 0.0
      ? std::clamp(
          ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / length_sq, 0.0, 1.0)
      : 0.0;
  return std::hypot(start.x() + along * dx - point.x(), start.y() + along * dy - point.y());
}

// The largest gap between the boxes projected on their side directions: positive when an axis
// separates them, else minus their smallest penetration depth.
double separation(const BoxCorners & first, const BoxCorners & second)
{
  const std::array<Point2d, 4> raw_axes{
    Point2d{first[0].x() - first[1].x(), first[0].y() - first[1].y()},
    Point2d{first[1].x() - first[2].x(), first[1].y() - first[2].y()},
    Point2d{second[0].x() - second[1].x(), second[0].y() - second[1].y()},
    Point2d{second[1].x() - second[2].x(), second[1].y() - second[2].y()}};
  double largest = -std::numeric_limits<double>::infinity();
  for (const auto & raw : raw_axes) {
    const double norm = std::max(std::hypot(raw.x(), raw.y()), 1.0e-6);
    const double ax = raw.x() / norm;
    const double ay = raw.y() / norm;
    const auto extent = [&](const BoxCorners & box) {
      double low = std::numeric_limits<double>::infinity();
      double high = -std::numeric_limits<double>::infinity();
      for (const auto & corner : box) {
        const double value = ax * corner.x() + ay * corner.y();
        low = std::min(low, value);
        high = std::max(high, value);
      }
      return std::make_pair(low, high);
    };
    const auto [first_low, first_high] = extent(first);
    const auto [second_low, second_high] = extent(second);
    largest = std::max({largest, first_low - second_high, second_low - first_high});
  }
  return largest;
}

}  // namespace

std::string to_string(const RoadUserType type)
{
  switch (type) {
    case RoadUserType::PEDESTRIAN:
      return "pedestrian";
    case RoadUserType::BICYCLE:
      return "bicycle";
    case RoadUserType::VEHICLE:
    default:
      return "vehicle";
  }
}

std::size_t RoadUserColumns::column(const std::string & track_id) const
{
  const auto found = std::find(track_ids.begin(), track_ids.end(), track_id);
  return static_cast<std::size_t>(found - track_ids.begin());
}

RoadUserColumns road_user_columns(
  const ClosedLoopEpisode & episode, const std::size_t half_window_rows)
{
  RoadUserColumns columns;
  const auto rows = episode.executed_rows();
  std::map<std::string, std::size_t> index;
  for (const auto & track : episode.road_users) {
    const auto id = metrics::object_id_to_string(track.object_id, track.has_valid_object_id);
    if (index.count(id) == 0U) {
      index.emplace(id, columns.track_ids.size());
      columns.track_ids.push_back(id);
      columns.types.push_back(type_of(track.states.front().classification));
    }
  }
  const auto count = columns.track_ids.size();
  columns.valid.assign(rows, std::vector<char>(count, 0));
  columns.corners.assign(rows, std::vector<BoxCorners>(count));
  columns.xy.assign(rows, std::vector<Point2d>(count, Point2d{0.0, 0.0}));
  columns.yaw.assign(rows, std::vector<double>(count, 0.0));
  columns.speed.assign(rows, std::vector<double>(count, 0.0));
  columns.fitted.assign(rows, std::vector<char>(count, 0));
  for (const auto & track : episode.road_users) {
    const auto column =
      index.at(metrics::object_id_to_string(track.object_id, track.has_valid_object_id));
    const auto track_row = track_rows(episode, track);
    for (std::size_t i = 0; i < track.states.size(); ++i) {
      const auto row = track_row.at(i);
      if (row >= rows) {
        continue;
      }
      const auto & state = track.states.at(i);
      const double yaw = metrics::get_yaw(state.pose.orientation);
      const double half_length = state.shape.dimensions.x / 2.0;
      const double half_width = state.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER
                                  ? state.shape.dimensions.x / 2.0
                                  : state.shape.dimensions.y / 2.0;
      columns.valid.at(row).at(column) = 1;
      columns.xy.at(row).at(column) = Point2d{state.pose.position.x, state.pose.position.y};
      columns.yaw.at(row).at(column) = yaw;
      columns.speed.at(row).at(column) = state.twist.linear.x;
      columns.corners.at(row).at(column) = box_corners(
        state.pose.position.x, state.pose.position.y, yaw, half_length, half_length, half_width,
        half_width);
    }
  }
  for (std::size_t column = 0; column < count; ++column) {
    for (std::size_t row = 0; row < rows; ++row) {
      const std::size_t low = row >= half_window_rows ? row - half_window_rows : 0U;
      const std::size_t high = std::min(rows, row + half_window_rows + 1U);
      std::size_t seen = 0;
      for (std::size_t other = low; other < high; ++other) {
        seen += columns.valid.at(other).at(column) != 0 ? 1U : 0U;
      }
      columns.fitted.at(row).at(column) = columns.valid.at(row).at(column) != 0 && seen >= 2U;
    }
  }
  return columns;
}

BoxCorners ego_box(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return box_corners(
    pose.position.x, pose.position.y, metrics::get_yaw(pose.orientation),
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.max_lateral_offset_m, -vehicle_info.min_lateral_offset_m);
}

std::vector<BoxCorners> ego_boxes(
  const ClosedLoopEpisode & episode, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  std::vector<BoxCorners> boxes;
  boxes.reserve(episode.executed_rows());
  for (const auto & point : episode.ego.points) {
    boxes.push_back(ego_box(point.pose, vehicle_info));
  }
  return boxes;
}

double box_clearance(const BoxCorners & first, const BoxCorners & second)
{
  const double overlap = separation(first, second);
  if (overlap < 0.0) {
    return overlap;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t side = 0; side < 4U; ++side) {
    for (const auto & corner : first) {
      distance = std::min(
        distance, point_segment_distance(corner, second.at(side), second.at((side + 1U) % 4U)));
    }
    for (const auto & corner : second) {
      distance = std::min(
        distance, point_segment_distance(corner, first.at(side), first.at((side + 1U) % 4U)));
    }
  }
  return distance;
}

}  // namespace autoware::planning_data_analyzer::closed_loop
