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
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag,
    const std::shared_ptr<TrajectoryPoints> & points);

  void normalize(
    const double min, const double max, const size_t score_type, const bool flip = false);

  void weighting(const std::vector<double> & weight);

  auto total() const -> double { return total_; };

  bool feasible() const;

  bool ready() const;

  auto score(const SCORE & score_type) const -> double;

  auto scores() const -> std::vector<double> { return scores_; }

  auto points() const -> std::shared_ptr<TrajectoryPoints> { return points_; }

  auto tag() const -> std::string { return tag_; }

protected:
  void evaluate();

private:
  auto lateral_accel(const size_t idx) const -> double;

  auto longitudinal_jerk(const size_t idx) const -> double;

  auto minimum_ttc(const size_t idx) const -> double;

  auto travel_distance(const size_t idx) const -> double;

  auto lateral_deviation(const size_t idx) const -> double;

  auto trajectory_deviation(const size_t idx) const -> double;

  auto compress(const METRIC & metric_type) const -> double;

  std::vector<PredictedObjects::SharedPtr> objects_history;

  std::vector<std::vector<double>> values_;

  std::vector<double> scores_;

  std::shared_ptr<TrajectoryPoints> previous_;

  std::shared_ptr<TrajectoryPoints> points_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<Parameters> parameters_;

  std::string tag_;

  double total_;
};

class GroundTruth : public DataInterface
{
public:
  GroundTruth(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & prev_best_data,
    const std::shared_ptr<VehicleInfo> & vehicle_info,
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
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag,
    const std::shared_ptr<TrajectoryPoints> & points);
};

class Evaluator
{
public:
  Evaluator(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & prev_best_data,
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<Parameters> & parameters);

  void show() const;

  auto results() const -> std::vector<std::shared_ptr<DataInterface>> { return results_; }

  auto best(const std::vector<double> & weight) -> std::shared_ptr<DataInterface>;

  auto loss(const std::vector<double> & weight) -> double;

  auto get(const std::string & tag) const -> std::shared_ptr<DataInterface>;

private:
  void normalize();

  void weighting(const std::vector<double> & weight);

  auto best() const -> std::shared_ptr<DataInterface>;

  std::vector<std::shared_ptr<DataInterface>> results_;

  std::shared_ptr<Parameters> parameters_;
};

}  // namespace autoware::behavior_analyzer

#endif  // EVALUATION_HPP_
