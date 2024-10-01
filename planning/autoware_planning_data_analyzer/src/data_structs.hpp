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

#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include "type_alias.hpp"

#include <string>
#include <vector>

namespace autoware::behavior_analyzer
{
struct TOPIC
{
  static std::string TF;
  static std::string ODOMETRY;
  static std::string ACCELERATION;
  static std::string OBJECTS;
  static std::string TRAJECTORY;
  static std::string STEERING;
  static std::string ROUTE;
};

// struct FrenetPoint
// {
//   double length{0.0};    // longitudinal
//   double distance{0.0};  // lateral
// };

struct TargetStateParameters
{
  std::vector<double> lat_positions{};
  std::vector<double> lat_velocities{};
  std::vector<double> lat_accelerations{};
  std::vector<double> lon_positions{};
  std::vector<double> lon_velocities{};
  std::vector<double> lon_accelerations{};
};

// struct GridSearchParameters
// {
//   double min{0.0};
//   double max{1.0};
//   double resolusion{0.01};
//   double dt{1.0};
//   size_t thread_num{4};
// };

struct DataAugmentParameters
{
  size_t sample_num{20};

  double resolution{0.5};

  TargetStateParameters target_state{};
};

struct Result
{
  Result(
    const double w0, const double w1, const double w2, const double w3, const double w4,
    const double w5)
  : weight{w0, w1, w2, w3, w4, w5}
  {
  }
  std::shared_ptr<TrajectoryPoints> previous_points{nullptr};
  std::vector<double> weight;
  double loss{0.0};
};
}  // namespace autoware::behavior_analyzer

#endif  // DATA_STRUCTS_HPP_
