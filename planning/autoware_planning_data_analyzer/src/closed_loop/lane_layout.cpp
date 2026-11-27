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

#include "closed_loop/lane_layout.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

namespace bg = boost::geometry;

// The stretch (m) over which a lanelet's heading at its end, and the path's, are read.
constexpr double kHeadingChordM = 2.0;
// Points per full circle of a widened lane's rounded corners (shapely's default quad_segs = 16).
constexpr int kBufferPointsPerCircle = 64;

double rounded(const double value)
{
  return std::round(value * 10.0) / 10.0;
}

double wrapped(const double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

Polygon2d to_polygon(const lanelet::ConstLanelet & lanelet)
{
  Polygon2d polygon;
  for (const auto & point : lanelet.polygon2d()) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  bg::correct(polygon);
  return polygon;
}

Polygon2d to_polygon(const lanelet::ConstArea & area)
{
  Polygon2d polygon;
  for (const auto & point : area.outerBoundPolygon()) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  for (const auto & inner : area.innerBoundPolygons()) {
    autoware_utils_geometry::LinearRing2d ring;
    for (const auto & point : inner) {
      ring.emplace_back(point.x(), point.y());
    }
    polygon.inners().push_back(ring);
  }
  bg::correct(polygon);
  return polygon;
}

MultiPolygon2d union_all(const std::vector<const MultiPolygon2d *> & parts)
{
  MultiPolygon2d result;
  for (const auto * part : parts) {
    MultiPolygon2d merged;
    bg::union_(result, *part, merged);
    result = std::move(merged);
  }
  return result;
}

MultiPolygon2d widened(const MultiPolygon2d & shape, const double distance_m)
{
  if (distance_m <= 0.0) {
    return shape;
  }
  MultiPolygon2d result;
  bg::buffer(
    shape, result, bg::strategy::buffer::distance_symmetric<double>(distance_m),
    bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_round(kBufferPointsPerCircle),
    bg::strategy::buffer::end_round(kBufferPointsPerCircle),
    bg::strategy::buffer::point_circle(kBufferPointsPerCircle));
  return result;
}

double length(const LineString2d & line)
{
  return bg::length(line);
}

// The arc length of the point of line nearest to point.
double project(const LineString2d & line, const Point2d & point)
{
  double best_distance = std::numeric_limits<double>::infinity();
  double best_along = 0.0;
  double along = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i) {
    const Point2d & a = line.at(i - 1);
    const Point2d & b = line.at(i);
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double segment = std::hypot(dx, dy);
    double ratio = 0.0;
    if (segment > 0.0) {
      ratio = std::clamp(
        ((point.x() - a.x()) * dx + (point.y() - a.y()) * dy) / (segment * segment), 0.0, 1.0);
    }
    const double distance =
      std::hypot(a.x() + ratio * dx - point.x(), a.y() + ratio * dy - point.y());
    if (distance < best_distance) {
      best_distance = distance;
      best_along = along + ratio * segment;
    }
    along += segment;
  }
  return best_along;
}

// The point at arc length along line (clamped to its ends).
Point2d interpolate(const LineString2d & line, const double along)
{
  if (line.empty()) {
    return Point2d{0.0, 0.0};
  }
  double remaining = std::max(along, 0.0);
  for (std::size_t i = 1; i < line.size(); ++i) {
    const Point2d & a = line.at(i - 1);
    const Point2d & b = line.at(i);
    const double segment = std::hypot(b.x() - a.x(), b.y() - a.y());
    if (remaining <= segment && segment > 0.0) {
      const double ratio = remaining / segment;
      return Point2d{a.x() + ratio * (b.x() - a.x()), a.y() + ratio * (b.y() - a.y())};
    }
    remaining -= segment;
  }
  return line.back();
}

Point2d centre_of(const BoxCorners & box)
{
  double x = 0.0;
  double y = 0.0;
  for (const auto & corner : box) {
    x += corner.x();
    y += corner.y();
  }
  return Point2d{x / 4.0, y / 4.0};
}

Polygon2d box_polygon(const BoxCorners & box)
{
  Polygon2d polygon;
  for (const auto & corner : box) {
    polygon.outer().push_back(corner);
  }
  bg::correct(polygon);
  return polygon;
}

bool covers_all(const MultiPolygon2d & shape, const BoxCorners & box)
{
  return std::all_of(
    box.begin(), box.end(), [&](const Point2d & corner) { return bg::covered_by(corner, shape); });
}

}  // namespace

LaneLayout::LaneLayout(const lanelet::LaneletMapConstPtr & map)
{
  if (!map) {
    return;
  }
  std::vector<lanelet::ConstLanelet> lanelets;
  for (const auto & lanelet : map->laneletLayer) {
    const std::string subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, "");
    const std::string vehicle = lanelet.attributeOr("participant:vehicle", "");
    if (
      subtype == "crosswalk" || subtype == "walkway" || subtype == "bicycle_lane" ||
      vehicle == "no") {
      continue;
    }
    lanelets.push_back(lanelet);
  }
  std::sort(lanelets.begin(), lanelets.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.id() < rhs.id();
  });

  std::map<std::pair<lanelet::Id, lanelet::Id>, std::vector<lanelet::Id>> starts;
  for (const auto & lanelet : lanelets) {
    Lane lane;
    lane.id = lanelet.id();
    lane.polygon.push_back(to_polygon(lanelet));
    for (const auto & point : lanelet.centerline2d()) {
      lane.centreline.emplace_back(point.x(), point.y());
    }
    index_.emplace(lane.id, lanes_.size());
    ids_.push_back(lane.id);
    lanes_.push_back(std::move(lane));
    successors_[lanelet.id()];
    predecessors_[lanelet.id()];
    if (!lanelet.leftBound().empty() && !lanelet.rightBound().empty()) {
      starts[{lanelet.leftBound().front().id(), lanelet.rightBound().front().id()}].push_back(
        lanelet.id());
    }
  }
  for (const auto & lanelet : lanelets) {
    if (lanelet.leftBound().empty() || lanelet.rightBound().empty()) {
      continue;
    }
    const auto found =
      starts.find({lanelet.leftBound().back().id(), lanelet.rightBound().back().id()});
    if (found == starts.end()) {
      continue;
    }
    for (const auto after : found->second) {
      if (after != lanelet.id()) {
        successors_[lanelet.id()].push_back(after);
        predecessors_[after].push_back(lanelet.id());
      }
    }
  }

  for (const auto & lane : lanes_) {
    drivable_.insert(drivable_.end(), lane.polygon.begin(), lane.polygon.end());
  }
  for (const auto & area : map->areaLayer) {
    const std::string subtype = area.attributeOr(lanelet::AttributeName::Subtype, "");
    if (subtype == "intersection" || subtype == "road" || subtype == "parking_access") {
      drivable_.push_back(to_polygon(area));
    }
  }
}

std::vector<std::size_t> LaneLayout::lanes_containing(
  const Point2d & point, const bool on_edge_too) const
{
  std::vector<std::size_t> found;
  for (std::size_t index = 0; index < lanes_.size(); ++index) {
    const auto & polygon = lanes_.at(index).polygon;
    if (on_edge_too ? bg::covered_by(point, polygon) : bg::within(point, polygon)) {
      found.push_back(index);
    }
  }
  return found;
}

const std::vector<MultiPolygon2d> & LaneLayout::chains(const lanelet::Id lane_id) const
{
  const auto cached = chains_.find(lane_id);
  if (cached != chains_.end()) {
    return cached->second;
  }
  const auto paths = [&](const std::map<lanelet::Id, std::vector<lanelet::Id>> & links) {
    std::vector<std::vector<lanelet::Id>> grown_paths{{}};
    for (int depth = 0; depth < kLaneChainDepth; ++depth) {
      std::vector<std::vector<lanelet::Id>> grown;
      for (const auto & path : grown_paths) {
        const auto & following = links.at(path.empty() ? lane_id : path.back());
        if (following.empty()) {
          grown.push_back(path);
          continue;
        }
        for (const auto next : following) {
          auto longer = path;
          longer.push_back(next);
          grown.push_back(std::move(longer));
        }
      }
      grown_paths = std::move(grown);
    }
    return grown_paths;
  };
  std::vector<MultiPolygon2d> lanes;
  for (const auto & before : paths(predecessors_)) {
    for (const auto & after : paths(successors_)) {
      std::set<lanelet::Id> members{lane_id};
      members.insert(before.begin(), before.end());
      members.insert(after.begin(), after.end());
      std::vector<const MultiPolygon2d *> parts;
      for (const auto member : members) {
        parts.push_back(&lane(member).polygon);
      }
      lanes.push_back(union_all(parts));
    }
  }
  return chains_.emplace(lane_id, std::move(lanes)).first->second;
}

const std::vector<MultiPolygon2d> & LaneLayout::widened_chains(
  const lanelet::Id lane_id, const double tolerance_m) const
{
  const auto key = std::make_pair(lane_id, tolerance_m);
  const auto cached = widened_chains_.find(key);
  if (cached != widened_chains_.end()) {
    return cached->second;
  }
  std::vector<MultiPolygon2d> lanes;
  for (const auto & chain : chains(lane_id)) {
    lanes.push_back(widened(chain, tolerance_m));
  }
  return widened_chains_.emplace(key, std::move(lanes)).first->second;
}

LaneLayout::AreaFlags LaneLayout::flags(const BoxCorners & box) const
{
  AreaFlags flags;
  flags.off_road = !std::all_of(box.begin(), box.end(), [&](const Point2d & corner) {
    return std::any_of(drivable_.begin(), drivable_.end(), [&](const Polygon2d & area) {
      return bg::covered_by(corner, area);
    });
  });
  std::vector<lanelet::Id> touched;
  for (const auto & lane : lanes_) {
    if (std::any_of(box.begin(), box.end(), [&](const Point2d & corner) {
          return bg::covered_by(corner, lane.polygon);
        })) {
      touched.push_back(lane.id);
    }
  }
  if (touched.size() > 1U) {
    flags.straddles = true;
    for (const auto id : touched) {
      for (const auto & chain : chains(id)) {
        if (covers_all(chain, box)) {
          flags.straddles = false;
          return flags;
        }
      }
    }
  }
  return flags;
}

bool LaneLayout::on_road(const BoxCorners & box) const
{
  const auto polygon = box_polygon(box);
  return std::any_of(lanes_.begin(), lanes_.end(), [&](const Lane & lane) {
    return bg::intersects(polygon, lane.polygon);
  });
}

bool LaneLayout::same_way(const BoxCorners & box, const double yaw, const double tolerance_m) const
{
  const auto centre = centre_of(box);
  for (const auto index : lanes_containing(centre, true)) {
    const auto & lane = lanes_.at(index);
    const auto & line = lane.centreline;
    if (line.size() < 2U) {
      continue;
    }
    const double at = project(line, centre);
    const auto ahead_point = interpolate(line, std::min(at + 0.5, length(line)));
    const auto behind_point = interpolate(line, std::max(at - 0.5, 0.0));
    const double lane_heading =
      std::atan2(ahead_point.y() - behind_point.y(), ahead_point.x() - behind_point.x());
    if (std::abs(wrapped(lane_heading - yaw)) > M_PI_2) {
      continue;
    }
    for (const auto & chain : widened_chains(lane.id, tolerance_m)) {
      if (covers_all(chain, box)) {
        return true;
      }
    }
  }
  return false;
}

double LaneLayout::driven(const lanelet::Id id, const LineString2d & path) const
{
  autoware_utils_geometry::MultiLineString2d inside_part;
  bg::intersection(path, lane(id).polygon, inside_part);
  return rounded(bg::length(inside_part));
}

double LaneLayout::turn(const lanelet::Id before, const lanelet::Id after) const
{
  const auto & end = lane(before).centreline;
  const auto & next = lane(after).centreline;
  if (end.size() < 2U || next.size() < 2U) {
    return 0.0;
  }
  const double out = std::atan2(
    end.at(end.size() - 1).y() - end.at(end.size() - 2).y(),
    end.at(end.size() - 1).x() - end.at(end.size() - 2).x());
  const double on =
    std::atan2(next.back().y() - next.front().y(), next.back().x() - next.front().x());
  return std::abs(wrapped(on - out));
}

double LaneLayout::turn_into(const lanelet::Id before, const lanelet::Id after) const
{
  const auto & own = lane(before).centreline;
  const auto & next = lane(after).centreline;
  if (own.size() < 2U || next.size() < 2U) {
    return 0.0;
  }
  const double out = std::atan2(own.back().y() - own.front().y(), own.back().x() - own.front().x());
  const double on = std::atan2(next.at(1).y() - next.at(0).y(), next.at(1).x() - next.at(0).x());
  return std::abs(wrapped(on - out));
}

double LaneLayout::off_heading(
  const lanelet::Id id, const LineString2d & path, const bool forward) const
{
  const auto & centre = lane(id).centreline;
  if (centre.size() < 2U || path.size() < 2U) {
    return 0.0;
  }
  const double centre_length = length(centre);
  const double chord = std::min(kHeadingChordM, centre_length);
  Point2d a;
  Point2d b;
  double along = 0.0;
  if (forward) {
    a = interpolate(centre, centre_length - chord);
    b = centre.back();
    along = project(path, b);
  } else {
    a = centre.front();
    b = interpolate(centre, chord);
    along = project(path, a);
  }
  const auto p = interpolate(path, std::max(along - kHeadingChordM, 0.0));
  const auto q = interpolate(path, std::min(along + kHeadingChordM, length(path)));
  if ((a.x() == b.x() && a.y() == b.y()) || (p.x() == q.x() && p.y() == q.y())) {
    return 0.0;
  }
  const double lane_heading = std::atan2(b.y() - a.y(), b.x() - a.x());
  const double drive_heading = std::atan2(q.y() - p.y(), q.x() - p.x());
  return std::abs(wrapped(lane_heading - drive_heading));
}

double LaneLayout::ahead(
  const lanelet::Id id, const LineString2d & path, const double reach_m) const
{
  double furthest = 0.0;
  std::set<lanelet::Id> seen{id};
  std::vector<lanelet::Id> todo{id};
  while (!todo.empty()) {
    const auto current = todo.back();
    todo.pop_back();
    const auto & centre = lane(current).centreline;
    if (!centre.empty()) {
      furthest = std::max(furthest, project(path, centre.back()));
    }
    for (const auto following : successors_.at(current)) {
      if (seen.count(following) == 0U && bg::distance(lane(following).polygon, path) <= reach_m) {
        seen.insert(following);
        todo.push_back(following);
      }
    }
  }
  return rounded(furthest);
}

std::set<lanelet::Id> LaneLayout::along(
  const lanelet::Id id, const bool forward, const LineString2d & path, const double reach_m,
  const bool one_branch, const bool branch_ahead, const std::optional<double> heading_rad) const
{
  const auto & links = forward ? successors_ : predecessors_;
  std::set<lanelet::Id> seen{id};
  std::vector<lanelet::Id> todo{id};
  while (!todo.empty()) {
    const auto current = todo.back();
    todo.pop_back();
    std::vector<lanelet::Id> near;
    for (const auto following : links.at(current)) {
      if (
        seen.count(following) != 0U ||
        std::find(near.begin(), near.end(), following) != near.end() ||
        bg::distance(lane(following).polygon, path) > reach_m) {
        continue;
      }
      if (heading_rad.has_value() && off_heading(following, path, forward) > *heading_rad) {
        continue;
      }
      near.push_back(following);
    }
    if (one_branch && near.size() > 1U) {
      const auto key = [&](const lanelet::Id following) {
        const double turn_rad = forward ? turn(current, following) : turn_into(following, current);
        const double ahead_m = branch_ahead && forward ? ahead(following, path, reach_m) : 0.0;
        return std::make_tuple(-ahead_m, -driven(following, path), turn_rad);
      };
      lanelet::Id best = near.front();
      auto best_key = key(best);
      for (std::size_t i = 1; i < near.size(); ++i) {
        const auto candidate_key = key(near.at(i));
        if (candidate_key < best_key) {
          best = near.at(i);
          best_key = candidate_key;
        }
      }
      near = {best};
    }
    for (const auto following : near) {
      seen.insert(following);
      todo.push_back(following);
    }
  }
  return seen;
}

std::set<lanelet::Id> LaneLayout::onward(
  const std::set<lanelet::Id> & lanes, const int depth, const bool one_branch) const
{
  std::vector<lanelet::Id> ends;
  for (const auto id : lanes) {
    const auto & following = successors_.at(id);
    if (std::none_of(following.begin(), following.end(), [&](const lanelet::Id next) {
          return lanes.count(next) != 0U;
        })) {
      ends.push_back(id);
    }
  }
  std::set<lanelet::Id> found;
  for (int step = 0; step < depth; ++step) {
    std::vector<lanelet::Id> following_ends;
    for (const auto end : ends) {
      std::vector<lanelet::Id> options;
      for (const auto next : successors_.at(end)) {
        if (lanes.count(next) == 0U && found.count(next) == 0U) {
          options.push_back(next);
        }
      }
      if (one_branch && options.size() > 1U) {
        options = {*std::min_element(
          options.begin(), options.end(),
          [&](const auto lhs, const auto rhs) { return turn(end, lhs) < turn(end, rhs); })};
      }
      following_ends.insert(following_ends.end(), options.begin(), options.end());
    }
    ends = following_ends;
    found.insert(ends.begin(), ends.end());
  }
  return found;
}

std::optional<MultiPolygon2d> LaneLayout::grow(
  const std::vector<lanelet::Id> & seeds, const LineString2d & path, const BoxCorners & box,
  const StartLaneOptions & options) const
{
  const std::optional<double> heading_rad =
    options.heading_deg.has_value() ? std::optional<double>(*options.heading_deg * M_PI / 180.0)
                                    : std::nullopt;
  std::set<lanelet::Id> lanes;
  for (const auto seed : seeds) {
    const auto forward = along(
      seed, true, path, options.reach_m, options.one_branch, options.branch_ahead, heading_rad);
    const auto backward =
      along(seed, false, path, options.reach_m, options.one_branch, false, heading_rad);
    lanes.insert(forward.begin(), forward.end());
    lanes.insert(backward.begin(), backward.end());
  }
  if (lanes.empty()) {
    return std::nullopt;
  }
  if (options.successor_lanelets > 0) {
    const auto more = onward(lanes, options.successor_lanelets, options.one_branch);
    lanes.insert(more.begin(), more.end());
  }
  std::vector<const MultiPolygon2d *> parts;
  for (const auto id : lanes) {
    parts.push_back(&lane(id).polygon);
  }
  auto shape = widened(union_all(parts), options.tolerance_m);
  if (options.hold && !covers_all(shape, box)) {
    return std::nullopt;
  }
  return shape;
}

std::optional<MultiPolygon2d> LaneLayout::start_lane(
  const BoxCorners & box, const LineString2d & path, const StartLaneOptions & options) const
{
  const auto centre = centre_of(box);
  const auto closest = [&](const std::vector<std::size_t> & indices) {
    std::size_t best = indices.front();
    double best_driven = driven(lanes_.at(best).id, path);
    for (std::size_t i = 1; i < indices.size(); ++i) {
      const double value = driven(lanes_.at(indices.at(i)).id, path);
      if (value > best_driven) {
        best = indices.at(i);
        best_driven = value;
      }
    }
    return best;
  };
  const auto ids_of = [&](const std::vector<std::size_t> & indices) {
    std::vector<lanelet::Id> ids;
    for (const auto index : indices) {
      ids.push_back(lanes_.at(index).id);
    }
    return ids;
  };

  const auto under = lanes_containing(centre, false);
  if (!under.empty()) {
    const auto seeds = options.one_branch && under.size() > 1U
                         ? std::vector<lanelet::Id>{lanes_.at(closest(under)).id}
                         : ids_of(under);
    if (auto lane = grow(seeds, path, box, options)) {
      return lane;
    }
  }
  std::vector<std::pair<std::size_t, MultiPolygon2d>> held;
  for (const auto index : lanes_containing(centre, true)) {
    if (auto lane = grow({lanes_.at(index).id}, path, box, options)) {
      held.emplace_back(index, std::move(*lane));
    }
  }
  if (held.empty()) {
    return std::nullopt;
  }
  if (options.one_branch && held.size() > 1U) {
    std::vector<std::size_t> indices;
    for (const auto & [index, _] : held) {
      indices.push_back(index);
    }
    const auto best = closest(indices);
    for (auto & [index, lane] : held) {
      if (index == best) {
        return lane;
      }
    }
  }
  std::vector<const MultiPolygon2d *> parts;
  for (const auto & [_, lane] : held) {
    parts.push_back(&lane);
  }
  return union_all(parts);
}

bool LaneLayout::inside(const BoxCorners & box, const MultiPolygon2d & lane)
{
  return covers_all(lane, box);
}

double LaneLayout::share_inside(const BoxCorners & box, const MultiPolygon2d & lane)
{
  const auto polygon = box_polygon(box);
  const double area = bg::area(polygon);
  if (area <= 0.0) {
    return 0.0;
  }
  MultiPolygon2d held;
  bg::intersection(polygon, lane, held);
  return bg::area(held) / area;
}

BoxCorners box_corners(
  const double x, const double y, const double yaw, const double front_m, const double rear_m,
  const double left_m, const double right_m)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const auto at = [&](const double forward, const double leftward) {
    return Point2d{x + c * forward - s * leftward, y + s * forward + c * leftward};
  };
  return {at(front_m, left_m), at(-rear_m, left_m), at(-rear_m, -right_m), at(front_m, -right_m)};
}

double distance_to_path(const Point2d & point, const LineString2d & path)
{
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (path.size() == 1U) {
    return std::hypot(point.x() - path.front().x(), point.y() - path.front().y());
  }
  return bg::distance(point, path);
}

double path_progress(const Point2d & point, const LineString2d & path)
{
  return project(path, point);
}

}  // namespace autoware::planning_data_analyzer::closed_loop
