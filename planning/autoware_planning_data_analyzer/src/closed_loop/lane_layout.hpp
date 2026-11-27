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

#ifndef CLOSED_LOOP__LANE_LAYOUT_HPP_
#define CLOSED_LOOP__LANE_LAYOUT_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <array>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

// The four corners of a box in the map frame.
using BoxCorners = std::array<Point2d, 4>;

// Lanelets before and after one lanelet that still belong to its lane.
constexpr int kLaneChainDepth = 2;

struct StartLaneOptions
{
  // Lanelets join the lane while they lie within this of the path (m).
  double reach_m{4.0};
  // The lane is widened by this (m): a wide vehicle in a narrow lane pokes over its lines.
  double tolerance_m{0.1};
  // Up to this many lanelets past the lane's ends join it, whatever their distance to the path.
  int successor_lanelets{0};
  // One lane, not every branch: where several linked lanelets qualify, only the one the path
  // runs most in (then the one that turns least).
  bool one_branch{false};
  // With one_branch, going forward: first the branch whose lanelets go on furthest along the path.
  bool branch_ahead{false};
  // A lanelet joins only when it heads within this of the path at its far end (deg).
  std::optional<double> heading_deg;
  // False: the lane under the box's centre even when it does not hold the box wholly.
  bool hold{true};
};

// The map's driving lanes as the reviewed closed-loop metrics read them. Lanes are the lanelets a
// vehicle may use (not crosswalks, walkways or bicycle lanes). A lanelet follows another when its
// bounds start at the points the other's end at; a lane is a lanelet with up to kLaneChainDepth of
// them before and after.
class LaneLayout
{
public:
  explicit LaneLayout(const lanelet::LaneletMapConstPtr & map);

  struct AreaFlags
  {
    // Corners in more than one lanelet, and no lane holds all four.
    bool straddles{false};
    // A corner outside the drivable area (the lanes and the intersection, road and
    // parking_access areas).
    bool off_road{false};
  };
  AreaFlags flags(const BoxCorners & box) const;

  // Every lane through the lanelet, one polygon each.
  const std::vector<MultiPolygon2d> & chains(lanelet::Id lane_id) const;
  // The same lanes widened by tolerance_m.
  const std::vector<MultiPolygon2d> & widened_chains(lanelet::Id lane_id, double tolerance_m) const;

  // The box touches a lanelet.
  bool on_road(const BoxCorners & box) const;

  // The box lies wholly in one lane that runs its way (heading yaw): a lane through a lanelet
  // under (or with on an edge) the box's centre, widened by tolerance_m, whose centreline heads
  // within 90 deg of yaw at the point nearest the centre.
  bool same_way(const BoxCorners & box, double yaw, double tolerance_m) const;

  // The lane a box starts in along a path (see StartLaneOptions); none when no lane holds it.
  std::optional<MultiPolygon2d> start_lane(
    const BoxCorners & box, const LineString2d & path, const StartLaneOptions & options) const;

  static bool inside(const BoxCorners & box, const MultiPolygon2d & lane);
  static double share_inside(const BoxCorners & box, const MultiPolygon2d & lane);

  std::size_t lane_count() const { return ids_.size(); }

private:
  struct Lane
  {
    lanelet::Id id{lanelet::InvalId};
    MultiPolygon2d polygon;
    LineString2d centreline;
  };

  const Lane & lane(lanelet::Id id) const { return lanes_.at(index_.at(id)); }
  std::vector<std::size_t> lanes_containing(const Point2d & point, bool on_edge_too) const;
  double driven(lanelet::Id id, const LineString2d & path) const;
  double turn(lanelet::Id before, lanelet::Id after) const;
  double turn_into(lanelet::Id before, lanelet::Id after) const;
  double off_heading(lanelet::Id id, const LineString2d & path, bool forward) const;
  double ahead(lanelet::Id id, const LineString2d & path, double reach_m) const;
  std::set<lanelet::Id> along(
    lanelet::Id id, bool forward, const LineString2d & path, double reach_m, bool one_branch,
    bool branch_ahead, std::optional<double> heading_rad) const;
  std::set<lanelet::Id> onward(
    const std::set<lanelet::Id> & lanes, int depth, bool one_branch) const;
  std::optional<MultiPolygon2d> grow(
    const std::vector<lanelet::Id> & seeds, const LineString2d & path, const BoxCorners & box,
    const StartLaneOptions & options) const;

  std::vector<Lane> lanes_;
  std::vector<lanelet::Id> ids_;
  std::map<lanelet::Id, std::size_t> index_;
  std::map<lanelet::Id, std::vector<lanelet::Id>> successors_;
  std::map<lanelet::Id, std::vector<lanelet::Id>> predecessors_;
  std::vector<Polygon2d> drivable_;
  mutable std::map<lanelet::Id, std::vector<MultiPolygon2d>> chains_;
  mutable std::map<std::pair<lanelet::Id, double>, std::vector<MultiPolygon2d>> widened_chains_;
};

// Box corners (front-left, rear-left, rear-right, front-right) of a vehicle pose.
BoxCorners box_corners(
  double x, double y, double yaw, double front_m, double rear_m, double left_m, double right_m);

// The distance from a point to a polyline (to its first or last point beyond its ends).
double distance_to_path(const Point2d & point, const LineString2d & path);

// How far along a polyline its point nearest to point is (m): 0 before its start, its length past
// its end.
double path_progress(const Point2d & point, const LineString2d & path);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__LANE_LAYOUT_HPP_
