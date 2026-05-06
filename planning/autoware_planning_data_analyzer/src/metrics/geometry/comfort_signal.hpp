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

#ifndef METRICS__COMFORT_SIGNAL_HPP_
#define METRICS__COMFORT_SIGNAL_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct ComfortSignalInput
{
  double time_s{0.0};
  geometry_msgs::msg::Pose pose;
  double longitudinal_acceleration_mps2{0.0};
  double lateral_acceleration_mps2{0.0};
};

struct ComfortSignals
{
  std::vector<double> longitudinal_accelerations;
  std::vector<double> lateral_accelerations;
  std::vector<double> acceleration_magnitudes;
  std::vector<double> longitudinal_jerks;
  std::vector<double> lateral_jerks;
  std::vector<double> jerk_magnitudes;
  std::vector<double> yaw_rates;
  std::vector<double> yaw_accelerations;
};

ComfortSignals compute_comfort_signals(
  const std::vector<ComfortSignalInput> & inputs, const double time_offset_s = 0.0);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__COMFORT_SIGNAL_HPP_
