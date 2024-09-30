// Copyright 2024 TIER IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "autoware_frenet_planner/frenet_planner.hpp"
#include "autoware_path_sampler/prepare_inputs.hpp"
#include "autoware_path_sampler/utils/trajectory_utils.hpp"
#include "evaluation.hpp"
#include "type_alias.hpp"

#include <memory>
#include <vector>

namespace autoware::behavior_analyzer::utils
{
auto convertToTrajectoryPoints(
  const autoware::sampler_common::Trajectory & trajectory,
  const std::shared_ptr<VehicleInfo> & vehicle_info, const double z)
  -> std::vector<TrajectoryPoint>;

template <class T>
auto convertToFrenetPoint(const T & points, const Point & search_point_geom, const size_t seg_idx)
  -> FrenetPoint;

auto prepareSamplingParameters(
  const autoware::sampler_common::Configuration & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline,
  [[maybe_unused]] const double trajectory_length,
  const std::shared_ptr<TargetStateParameters> & parameters)
  -> autoware::frenet_planner::SamplingParameters;

auto resampling(
  const Trajectory & trajectory, const Pose & p_ego, const size_t resample_num,
  const double time_resolution) -> std::vector<TrajectoryPoint>;

auto sampling(
  const Trajectory & trajectory, const Pose & p_ego, const double v_ego, const double a_ego,
  const std::shared_ptr<VehicleInfo> & vehicle_info,
  const std::shared_ptr<EvaluatorParameters> & parameters)
  -> std::vector<std::vector<TrajectoryPoint>>;

auto to_marker(
  const std::shared_ptr<trajectory_selector::trajectory_evaluator::DataInterface> & data,
  const trajectory_selector::trajectory_evaluator::SCORE & score_type, const size_t id) -> Marker;
}  // namespace autoware::behavior_analyzer::utils

#endif  // UTILS_HPP_
