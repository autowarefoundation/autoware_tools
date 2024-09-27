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

#ifndef EVALUATION_HPP_
#define EVALUATION_HPP_

#include "bag_handler.hpp"
#include "type_alias.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_analyzer
{

class DataInterface
{
public:
  DataInterface(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<TrajectoryPoints> & previous,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag,
    const std::shared_ptr<TrajectoryPoints> & points);

  void calculate();

  void normalize(
    const double min, const double max, const size_t score_type, const bool flip = false);

  auto to_1d(const METRIC & metric_type) const -> double;

  auto total(const std::vector<double> & weight) const -> double;

  double get(const SCORE & score_type) const;

  auto score() const -> std::vector<double> { return scores; }

  double lateral_accel(const size_t idx) const;

  double longitudinal_jerk(const size_t idx) const;

  double minimum_ttc(const size_t idx) const;

  double travel_distance(const size_t idx) const;

  double lateral_deviation(const size_t idx) const;

  double trajectory_deviation(const size_t idx) const;

  bool feasible() const;

  bool ready() const;

  auto points() const -> std::shared_ptr<TrajectoryPoints> { return points_; }

  std::shared_ptr<TrajectoryPoints> previous_;

  std::vector<PredictedObjects::SharedPtr> objects_history;

  std::vector<std::vector<double>> values;

  std::vector<double> scores;

  vehicle_info_utils::VehicleInfo vehicle_info;

  std::shared_ptr<RouteHandler> route_handler;

  std::shared_ptr<Parameters> parameters;

  std::shared_ptr<TrajectoryPoints> points_;

  std::string tag{""};
};

class GroundTruth : public DataInterface
{
public:
  GroundTruth(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & prev_best_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag);

private:
  static auto to_points(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<Parameters> & parameters)
    -> std::shared_ptr<TrajectoryPoints>;
};

class TrajectoryData : public DataInterface
{
public:
  TrajectoryData(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & prev_best_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag,
    const std::shared_ptr<TrajectoryPoints> & points);
};

class Evaluator
{
public:
  Evaluator(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & prev_best_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters);

  void show();

  void normalize_scores();

  auto loss(const std::vector<double> & weight) const -> double;

  auto get(const SCORE & score_type) const -> std::vector<double>;

  auto best(const std::vector<double> & weight) const -> std::shared_ptr<DataInterface>;

  auto get(const std::string & tag) const -> std::shared_ptr<DataInterface>;

  std::vector<std::shared_ptr<DataInterface>> data_set;

  std::shared_ptr<RouteHandler> route_handler;

  std::shared_ptr<Parameters> parameters;
};

}  // namespace autoware::behavior_analyzer

#endif  // EVALUATION_HPP_
