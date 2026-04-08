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

#include "trajectory_message_adapter.hpp"

#include <autoware_internal_planning_msgs/msg/generator_info.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{
namespace
{
double durationToSec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

/// Menger curvature on three consecutive XY points: magnitude from circumradius, sign from
/// triangle orientation.
double mengerCurvatureSigned2D(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  const double abx = p1.x - p0.x;
  const double aby = p1.y - p0.y;
  const double bcx = p2.x - p1.x;
  const double bcy = p2.y - p1.y;
  const double acx = p2.x - p0.x;
  const double acy = p2.y - p0.y;
  const double a = std::hypot(abx, aby);
  const double b = std::hypot(bcx, bcy);
  const double c = std::hypot(acx, acy);
  if (a < 1e-9 || b < 1e-9 || c < 1e-9) {
    return 0.0;
  }
  const double cross = abx * bcy - aby * bcx;
  const double area = 0.5 * std::abs(cross);
  const double kappa_mag = 4.0 * area / (a * b * c);
  const double sign = cross >= 0.0 ? 1.0 : -1.0;
  return sign * kappa_mag;
}

std::string lookupGeneratorName(
  const std::vector<autoware_internal_planning_msgs::msg::GeneratorInfo> & infos,
  const unique_identifier_msgs::msg::UUID & id)
{
  for (const auto & g : infos) {
    if (g.generator_id.uuid.size() != 16U || id.uuid.size() != 16U) {
      continue;
    }
    if (std::equal(g.generator_id.uuid.begin(), g.generator_id.uuid.end(), id.uuid.begin())) {
      return g.generator_name.data;
    }
  }
  return {};
}

}  // namespace

std::vector<SeriesPoint> buildSeriesFromTrajectoryPoints(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points)
{
  const size_t n = points.size();
  std::vector<SeriesPoint> out(n);
  if (n == 0U) {
    return out;
  }

  double arc = 0.0;
  for (size_t i = 0; i < n; ++i) {
    out[i].time_from_start_sec = durationToSec(points[i].time_from_start);
    out[i].longitudinal_velocity_mps = static_cast<double>(points[i].longitudinal_velocity_mps);
    out[i].acceleration_mps2 = static_cast<double>(points[i].acceleration_mps2);
    if (i > 0U) {
      const double dx = points[i].pose.position.x - points[i - 1U].pose.position.x;
      const double dy = points[i].pose.position.y - points[i - 1U].pose.position.y;
      arc += std::hypot(dx, dy);
    }
    out[i].arc_length_m = arc;
  }

  std::vector<double> kappa(n, 0.0);
  if (n >= 3U) {
    for (size_t i = 1U; i + 1U < n; ++i) {
      kappa[i] = mengerCurvatureSigned2D(
        points[i - 1U].pose.position, points[i].pose.position, points[i + 1U].pose.position);
    }
    // Endpoints lack a full triple: reuse the neighbor interior estimate for a stable open
    // polyline.
    kappa[0] = kappa[1];
    kappa[n - 1U] = kappa[n - 2U];
  }

  for (size_t i = 0; i < n; ++i) {
    out[i].curvature_1pm = kappa[i];
    const double v = out[i].longitudinal_velocity_mps;
    out[i].lateral_acceleration_mps2 = v * v * kappa[i];
  }

  return out;
}

std::vector<TrajectorySeriesData> trajectoryMsgToSeries(
  const autoware_planning_msgs::msg::Trajectory & msg, const std::string & topic_name)
{
  TrajectorySeriesData series;
  series.topic = topic_name;
  series.key = "trajectory";
  series.label = topic_name + " (Trajectory)";
  series.points = buildSeriesFromTrajectoryPoints(msg.points);
  return {std::move(series)};
}

std::vector<TrajectorySeriesData> scoredCandidateTrajectoriesToSeries(
  const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories & msg,
  const std::string & topic_name)
{
  std::vector<TrajectorySeriesData> out;
  out.reserve(msg.scored_candidate_trajectories.size());
  for (size_t i = 0; i < msg.scored_candidate_trajectories.size(); ++i) {
    const auto & sct = msg.scored_candidate_trajectories[i];
    TrajectorySeriesData series;
    series.topic = topic_name;
    std::ostringstream key;
    key << "candidate_" << i;
    series.key = key.str();

    std::string gen =
      lookupGeneratorName(msg.generator_info, sct.candidate_trajectory.generator_id);
    if (gen.empty()) {
      gen = "candidate_" + std::to_string(i);
    }

    std::ostringstream label;
    label << topic_name << " [" << gen << "] score=" << static_cast<double>(sct.score);
    series.label = label.str();

    series.points = buildSeriesFromTrajectoryPoints(sct.candidate_trajectory.points);
    out.push_back(std::move(series));
  }
  return out;
}

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin
