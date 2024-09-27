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

struct CommonData
{
  CommonData(
    const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag);

  void calculate();

  void normalize(
    const double min, const double max, const size_t score_type, const bool flip = false);

  auto to_1d(const METRIC & metric_type) const -> double;

  auto total(const std::vector<double> & weight) const -> double;

  double get(const SCORE & score_type) const;

  auto score() const -> std::vector<double> { return scores; }

  virtual double lateral_accel(const size_t idx) const = 0;

  virtual double longitudinal_jerk(const size_t idx) const = 0;

  virtual double minimum_ttc(const size_t idx) const = 0;

  virtual double travel_distance(const size_t idx) const = 0;

  virtual double lateral_deviation(const size_t idx) const = 0;

  virtual double trajectory_deviation(const size_t idx) const = 0;

  virtual bool feasible() const = 0;

  virtual bool ready() const = 0;

  std::vector<PredictedObjects::SharedPtr> objects_history;

  std::vector<std::vector<double>> values;

  std::vector<double> scores;

  vehicle_info_utils::VehicleInfo vehicle_info;

  std::shared_ptr<RouteHandler> route_handler;

  std::shared_ptr<Parameters> parameters;

  std::string tag{""};
};

struct ManualDrivingData : CommonData
{
  ManualDrivingData(
    const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters);

  double lateral_accel(const size_t idx) const override;

  double longitudinal_jerk(const size_t idx) const override;

  double minimum_ttc(const size_t idx) const override;

  double travel_distance(const size_t idx) const override;

  double lateral_deviation(const size_t idx) const override;

  double trajectory_deviation(const size_t idx) const override;

  bool feasible() const override { return true; }

  bool ready() const override;

  std::vector<Odometry::SharedPtr> odometry_history;
  std::vector<AccelWithCovarianceStamped::SharedPtr> accel_history;
  std::vector<SteeringReport::SharedPtr> steer_history;
};

struct TrajectoryData : CommonData
{
  TrajectoryData(
    const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag,
    const std::vector<TrajectoryPoint> & points, const std::optional<TrajectoryPoints> & t_best);

  double lateral_accel(const size_t idx) const override;

  double longitudinal_jerk(const size_t idx) const override;

  double minimum_ttc(const size_t idx) const override;

  double travel_distance(const size_t idx) const override;

  double lateral_deviation(const size_t idx) const override;

  double trajectory_deviation(const size_t idx) const override;

  bool feasible() const override;

  bool ready() const override;

  std::vector<TrajectoryPoint> points;

  std::optional<TrajectoryPoints> t_best;
};

struct SamplingTrajectoryData
{
  SamplingTrajectoryData(
    const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::optional<TrajectoryPoints> & t_best);

  auto best(const std::vector<double> & weight) const -> std::optional<TrajectoryData>
  {
    auto sort_by_score = data;

    std::sort(
      sort_by_score.begin(), sort_by_score.end(),
      [&weight](const auto & a, const auto & b) { return a.total(weight) > b.total(weight); });

    const auto itr = std::remove_if(
      sort_by_score.begin(), sort_by_score.end(), [](const auto & d) { return !d.feasible(); });

    sort_by_score.erase(itr, sort_by_score.end());

    if (sort_by_score.empty()) return std::nullopt;

    return sort_by_score.front();
  }

  auto autoware() const -> std::optional<TrajectoryData>
  {
    const auto itr = std::find_if(data.begin(), data.end(), [](const auto & trajectory) {
      return trajectory.tag == "autoware";
    });
    if (itr == data.end()) return std::nullopt;
    return *itr;
  }

  std::vector<TrajectoryData> data;
};

struct DataSet
{
  DataSet(
    const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::optional<TrajectoryPoints> & t_best);

  void show();

  void normalize_scores();

  auto loss(const std::vector<double> & weight) const -> double;

  auto get(const SCORE & score_type) const -> std::vector<double>;

  ManualDrivingData manual;

  SamplingTrajectoryData sampling;

  std::shared_ptr<RouteHandler> route_handler;

  std::shared_ptr<Parameters> parameters;
};

}  // namespace autoware::behavior_analyzer

#endif  // EVALUATION_HPP_
