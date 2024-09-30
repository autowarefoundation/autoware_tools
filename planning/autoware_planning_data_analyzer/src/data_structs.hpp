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

enum class METRIC {
  LATERAL_ACCEL = 0,
  LONGITUDINAL_JERK = 1,
  TRAVEL_DISTANCE = 2,
  MINIMUM_TTC = 3,
  LATERAL_DEVIATION = 4,
  TRAJECTORY_DEVIATION = 5,
  SIZE
};

enum class SCORE {
  LATERAL_COMFORTABILITY = 0,
  LONGITUDINAL_COMFORTABILITY = 1,
  EFFICIENCY = 2,
  SAFETY = 3,
  ACHIEVABILITY = 4,
  CONSISTENCY = 5,
  SIZE
};

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

struct FrenetPoint
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

struct TargetStateParameters
{
  std::vector<double> lat_positions{};
  std::vector<double> lat_velocities{};
  std::vector<double> lat_accelerations{};
  std::vector<double> lon_positions{};
  std::vector<double> lon_velocities{};
  std::vector<double> lon_accelerations{};
};

struct GridSearchParameters
{
  double min{0.0};
  double max{1.0};
  double resolusion{0.01};
  double dt{1.0};
  size_t thread_num{4};
};

struct EvaluatorParameters
{
  size_t resample_num{20};
  double time_resolution{0.5};
  GridSearchParameters grid_seach{};
  TargetStateParameters target_state{};
};

struct SelectorParameters
{
  explicit SelectorParameters(const size_t sample_num)
  : time_decay_weight(static_cast<size_t>(METRIC::SIZE), std::vector<double>(sample_num, 0.0)),
    score_weight(static_cast<size_t>(SCORE::SIZE), 0.0)
  {
  }

  std::vector<std::vector<double>> time_decay_weight;
  std::vector<double> score_weight;
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

struct CoreData
{
  CoreData(
    const std::shared_ptr<TrajectoryPoints> & points,
    const std::shared_ptr<TrajectoryPoints> & previous_points,
    const std::shared_ptr<PredictedObjects> & objects, const std::string & tag)
  : points{points}, previous_points{previous_points}, objects{objects}, tag{tag}
  {
  }

  std::shared_ptr<TrajectoryPoints> points;

  std::shared_ptr<TrajectoryPoints> previous_points;

  std::shared_ptr<PredictedObjects> objects;

  std::string tag;
};

struct DataSet
{
  std::shared_ptr<TrajectoryPoints> points;
};

}  // namespace autoware::behavior_analyzer

#endif  // DATA_STRUCTS_HPP_
