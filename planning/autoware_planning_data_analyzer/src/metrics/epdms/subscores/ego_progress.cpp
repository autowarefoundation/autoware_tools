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

#include "../../geometry/metric_utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

std::optional<double> calculate_raw_progress_m(
  const Trajectory & trajectory, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (trajectory.points.size() < 2U) {
    return std::nullopt;
  }

  const auto route_lanelets = collect_route_relevant_lanelets(trajectory, route_handler);
  if (route_lanelets.empty()) {
    return std::nullopt;
  }
  const auto lanelet_map_ptr = route_handler->getLaneletMapPtr();
  if (!lanelet_map_ptr) {
    return std::nullopt;
  }

  const auto start_arc =
    autoware::experimental::lanelet2_utils::get_arc_coordinates_on_ego_centerline(
      route_lanelets, trajectory.points.front().pose, lanelet_map_ptr);
  const auto end_arc =
    autoware::experimental::lanelet2_utils::get_arc_coordinates_on_ego_centerline(
      route_lanelets, trajectory.points.back().pose, lanelet_map_ptr);

  return std::max(end_arc.length - start_arc.length, 0.0);
}

Trajectory to_trajectory(
  const autoware_internal_planning_msgs::msg::CandidateTrajectory & candidate_trajectory)
{
  Trajectory trajectory;
  trajectory.header = candidate_trajectory.header;
  trajectory.points = candidate_trajectory.points;
  return trajectory;
}

double calculate_trajectory_similarity(const Trajectory & lhs, const Trajectory & rhs)
{
  if (lhs.points.empty() || rhs.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto shared_size = std::min(lhs.points.size(), rhs.points.size());
  if (shared_size == 0U) {
    return std::numeric_limits<double>::max();
  }

  double total_distance = 0.0;
  for (size_t i = 0; i < shared_size; ++i) {
    const auto & lp = lhs.points.at(i).pose.position;
    const auto & rp = rhs.points.at(i).pose.position;
    total_distance += std::hypot(lp.x - rp.x, lp.y - rp.y);
  }

  // Normalize by the shared prefix length so candidate matching is not biased toward
  // trajectories with more points in common.
  total_distance /= static_cast<double>(shared_size);
  total_distance += std::abs(static_cast<double>(lhs.points.size()) - rhs.points.size());
  return total_distance;
}

std::optional<size_t> find_selected_candidate_index(
  const Trajectory & selected_trajectory, const CandidateTrajectories & candidate_trajectories)
{
  if (candidate_trajectories.candidate_trajectories.empty()) {
    return std::nullopt;
  }

  size_t best_index = 0U;
  double best_similarity = std::numeric_limits<double>::max();

  for (size_t i = 0; i < candidate_trajectories.candidate_trajectories.size(); ++i) {
    const auto candidate = to_trajectory(candidate_trajectories.candidate_trajectories.at(i));
    const double similarity = calculate_trajectory_similarity(selected_trajectory, candidate);
    if (similarity < best_similarity) {
      best_similarity = similarity;
      best_index = i;
    }
  }

  return best_index;
}

}  // namespace

EgoProgressResult calculate_ego_progress(
  const std::shared_ptr<Trajectory> & selected_trajectory,
  const std::shared_ptr<CandidateTrajectories> & candidate_trajectories,
  const std::shared_ptr<RouteHandler> & route_handler)
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

  const auto selected_raw_progress = calculate_raw_progress_m(*selected_trajectory, route_handler);
  if (!selected_raw_progress.has_value()) {
    result.reason = "unavailable_no_route_lanelets";
    return result;
  }
  result.raw_progress_m = selected_raw_progress.value();

  if (!candidate_trajectories) {
    result.reason = "unavailable_no_candidate_trajectories";
    return result;
  }
  if (candidate_trajectories->candidate_trajectories.empty()) {
    result.reason = "unavailable_empty_candidate_trajectories";
    return result;
  }

  const auto selected_candidate_index =
    find_selected_candidate_index(*selected_trajectory, *candidate_trajectories);
  if (!selected_candidate_index.has_value()) {
    result.reason = "unavailable_no_matching_selected_candidate";
    return result;
  }

  double best_raw_progress_m = 0.0;
  bool has_valid_candidate = false;
  std::optional<double> selected_candidate_raw_progress;

  for (size_t i = 0; i < candidate_trajectories->candidate_trajectories.size(); ++i) {
    const auto candidate = to_trajectory(candidate_trajectories->candidate_trajectories.at(i));
    const auto candidate_raw_progress = calculate_raw_progress_m(candidate, route_handler);
    if (!candidate_raw_progress.has_value()) {
      continue;
    }

    has_valid_candidate = true;
    best_raw_progress_m = std::max(best_raw_progress_m, candidate_raw_progress.value());
    if (i == selected_candidate_index.value()) {
      selected_candidate_raw_progress = candidate_raw_progress;
    }
  }

  if (!has_valid_candidate) {
    result.reason = "unavailable_no_valid_candidate_progress";
    return result;
  }
  if (!selected_candidate_raw_progress.has_value()) {
    result.reason = "unavailable_selected_candidate_invalid";
    return result;
  }

  result.raw_progress_m = selected_candidate_raw_progress.value();
  result.best_raw_progress_m = best_raw_progress_m;
  result.available = true;
  result.reason = "available";

  constexpr double kProgressDistanceThresholdM = 5.0;
  if (best_raw_progress_m > kProgressDistanceThresholdM) {
    result.score =
      std::clamp(selected_candidate_raw_progress.value() / best_raw_progress_m, 0.0, 1.0);
  } else {
    result.score = 1.0;
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
