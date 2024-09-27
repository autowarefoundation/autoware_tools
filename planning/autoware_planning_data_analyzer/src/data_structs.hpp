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

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_analyzer
{

enum class METRIC {
  LATERAL_ACCEL = 0,
  LONGITUDINAL_ACCEL = 1,
  LONGITUDINAL_JERK = 2,
  TRAVEL_DISTANCE = 3,
  MINIMUM_TTC = 4,
  LATERAL_DEVIATION = 5,
  TRAJECTORY_DEVIATION = 6,
  SIZE
};

enum class SCORE {
  LATERAL_COMFORTABILITY = 0,
  LONGITUDINAL_COMFORTABILITY = 1,
  EFFICIENCY = 2,
  SAFETY = 3,
  ACHIEVABILITY = 4,
  CONSISTENCY = 5,
  TOTAL = 6,
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

struct Parameters
{
  size_t resample_num{20};
  double time_resolution{0.5};
  std::vector<double> weight;
  GridSearchParameters grid_seach{};
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
  std::vector<double> weight;
  double loss{0.0};
};

struct BufferBase
{
  virtual bool ready() const = 0;
  virtual void remove_old_data(const rcutils_time_point_value_t now) = 0;
};

template <typename T>
struct Buffer : BufferBase
{
  std::vector<T> msgs;

  const double BUFFER_TIME = 20.0 * 1e9;

  bool ready() const override
  {
    if (msgs.empty()) {
      return false;
    }

    return rclcpp::Time(msgs.back().header.stamp).nanoseconds() -
             rclcpp::Time(msgs.front().header.stamp).nanoseconds() >
           BUFFER_TIME;
  }

  void remove_old_data(const rcutils_time_point_value_t now) override
  {
    const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() < now;
    });
    msgs.erase(itr, msgs.end());
  }

  void append(const T & msg) { msgs.push_back(msg); }

  auto get(const rcutils_time_point_value_t now) const -> typename T::SharedPtr
  {
    const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() > now;
    });

    if (itr == msgs.end()) {
      return nullptr;
    }

    return std::make_shared<T>(*itr);
  }
};

template <>
bool Buffer<SteeringReport>::ready() const;

template <>
bool Buffer<TFMessage>::ready() const;

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now);

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now);

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const
  -> SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr;

struct BagData
{
  explicit BagData(
    const rcutils_time_point_value_t timestamp, std::shared_ptr<RouteHandler> & route_handler)
  : timestamp{timestamp}, route_handler{route_handler}
  {
    buffers.emplace(TOPIC::TF, std::make_shared<Buffer<TFMessage>>());
    buffers.emplace(TOPIC::ODOMETRY, std::make_shared<Buffer<Odometry>>());
    buffers.emplace(TOPIC::ACCELERATION, std::make_shared<Buffer<AccelWithCovarianceStamped>>());
    buffers.emplace(TOPIC::TRAJECTORY, std::make_shared<Buffer<Trajectory>>());
    buffers.emplace(TOPIC::OBJECTS, std::make_shared<Buffer<PredictedObjects>>());
    buffers.emplace(TOPIC::STEERING, std::make_shared<Buffer<SteeringReport>>());
  }

  rcutils_time_point_value_t timestamp;

  std::map<std::string, std::shared_ptr<BufferBase>> buffers{};

  std::shared_ptr<RouteHandler> route_handler;

  void update(const rcutils_time_point_value_t dt)
  {
    timestamp += dt;
    remove_old_data();
  }

  void remove_old_data()
  {
    std::for_each(buffers.begin(), buffers.end(), [this](const auto & buffer) {
      buffer.second->remove_old_data(timestamp);
    });
  }

  bool ready() const
  {
    return std::all_of(
      buffers.begin(), buffers.end(), [](const auto & buffer) { return buffer.second->ready(); });
  }
};

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

  // double total(
  //   const double w0, const double w1, const double w2, const double w3, const double w4,
  //   const double w5) const;

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

  // auto best(
  //   const double w0, const double w1, const double w2, const double w3, const double w4,
  //   const double w5) const -> std::optional<TrajectoryData>
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
    const std::shared_ptr<Parameters> & parameters, const std::optional<TrajectoryPoints> & t_best)
  : manual{ManualDrivingData(bag_data, vehicle_info, parameters)},
    sampling{SamplingTrajectoryData(bag_data, vehicle_info, parameters, t_best)},
    route_handler{bag_data->route_handler},
    parameters{parameters}
  {
    normalize_scores();
  }

  void normalize_scores()
  {
    const auto range = [this](const size_t idx) {
      const auto min_itr = std::min_element(
        sampling.data.begin(), sampling.data.end(),
        [&idx](const auto & a, const auto & b) { return a.scores.at(idx) < b.scores.at(idx); });
      const auto max_itr = std::max_element(
        sampling.data.begin(), sampling.data.end(),
        [&idx](const auto & a, const auto & b) { return a.scores.at(idx) < b.scores.at(idx); });

      return std::make_pair(min_itr->scores.at(idx), max_itr->scores.at(idx));
    };

    const auto [s0_min, s0_max] = range(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY));
    const auto [s1_min, s1_max] = range(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY));
    const auto [s2_min, s2_max] = range(static_cast<size_t>(SCORE::EFFICIENCY));
    const auto [s3_min, s3_max] = range(static_cast<size_t>(SCORE::SAFETY));
    const auto [s4_min, s4_max] = range(static_cast<size_t>(SCORE::ACHIEVABILITY));
    const auto [s5_min, s5_max] = range(static_cast<size_t>(SCORE::CONSISTENCY));

    for (auto & data : sampling.data) {
      data.normalize(s0_min, s0_max, static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY), true);
      data.normalize(s1_min, s1_max, static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY), true);
      data.normalize(s2_min, s2_max, static_cast<size_t>(SCORE::EFFICIENCY));
      data.normalize(s3_min, s3_max, static_cast<size_t>(SCORE::SAFETY));
      data.normalize(s4_min, s4_max, static_cast<size_t>(SCORE::ACHIEVABILITY), true);
      data.normalize(s5_min, s5_max, static_cast<size_t>(SCORE::CONSISTENCY), true);
      data.scores.at(static_cast<size_t>(SCORE::TOTAL)) = data.total(parameters->weight);
    }

    const auto [total_min, total_max] = range(static_cast<size_t>(SCORE::TOTAL));

    for (auto & data : sampling.data) {
      data.normalize(total_min, total_max, static_cast<size_t>(SCORE::TOTAL));
    }
  }

  auto loss(const std::vector<double> & weight) const -> double
  {
    const auto best = sampling.best(weight);
    if (!best.has_value()) {
      return 0.0;
    }

    const auto min_size = std::min(manual.odometry_history.size(), best.value().points.size());

    double mse = 0.0;
    for (size_t i = 0; i < min_size; i++) {
      const auto & p1 = manual.odometry_history.at(i)->pose.pose;
      const auto & p2 = best.value().points.at(i);
      mse = (mse * i + autoware::universe_utils::calcSquaredDistance2d(p1, p2)) / (i + 1);
    }

    if (!std::isfinite(mse)) {
      throw std::logic_error("loss value is invalid.");
    }

    return mse;
  }

  void show()
  {
    const auto best = sampling.best(parameters->weight);

    if (!best.has_value()) {
      return;
    }

    double s0_ave = 0.0;
    double s0_dev = 0.0;
    double s1_ave = 0.0;
    double s1_dev = 0.0;
    double s2_ave = 0.0;
    double s2_dev = 0.0;
    double s3_ave = 0.0;
    double s3_dev = 0.0;
    double s4_ave = 0.0;
    double s4_dev = 0.0;
    double s5_ave = 0.0;
    double s5_dev = 0.0;

    const auto update = [](const double ave, const double dev, const double value, const size_t i) {
      const auto new_ave = (i * ave + value) / (i + 1);
      const auto new_dev =
        (i * (ave * ave + dev * dev) + value * value) / (i + 1) - new_ave * new_ave;
      return std::make_pair(new_ave, new_dev);
    };

    for (size_t i = 0; i < sampling.data.size(); i++) {
      const auto data = sampling.data.at(i);
      std::tie(s0_ave, s0_dev) = update(s0_ave, s0_dev, data.get(SCORE::LATERAL_COMFORTABILITY), i);
      std::tie(s1_ave, s1_dev) =
        update(s1_ave, s1_dev, data.get(SCORE::LONGITUDINAL_COMFORTABILITY), i);
      std::tie(s2_ave, s2_dev) = update(s2_ave, s2_dev, data.get(SCORE::EFFICIENCY), i);
      std::tie(s3_ave, s3_dev) = update(s3_ave, s3_dev, data.get(SCORE::SAFETY), i);
      std::tie(s4_ave, s4_dev) = update(s4_ave, s4_dev, data.get(SCORE::ACHIEVABILITY), i);
      std::tie(s5_ave, s5_dev) = update(s5_ave, s5_dev, data.get(SCORE::CONSISTENCY), i);
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "\n";
    // clang-format off
    ss << " tag               :" << best.value().tag << "\n";
    ss << " lat comfortability:" << best.value().get(SCORE::LATERAL_COMFORTABILITY)       << " mean:" << s0_ave << " std:" << std::sqrt(s0_dev) << "\n"; // NOLINT
    ss << " lon comfortability:" << best.value().get(SCORE::LONGITUDINAL_COMFORTABILITY)  << " mean:" << s1_ave << " std:" << std::sqrt(s1_dev) << "\n"; // NOLINT
    ss << " efficiency        :" << best.value().get(SCORE::EFFICIENCY)                   << " mean:" << s2_ave << " std:" << std::sqrt(s2_dev) << "\n"; // NOLINT
    ss << " safety            :" << best.value().get(SCORE::SAFETY)                       << " mean:" << s3_ave << " std:" << std::sqrt(s3_dev) << "\n"; // NOLINT
    ss << " achievability     :" << best.value().get(SCORE::ACHIEVABILITY)                << " mean:" << s4_ave << " std:" << std::sqrt(s4_dev) << "\n"; // NOLINT
    ss << " consistency       :" << best.value().get(SCORE::CONSISTENCY)                  << " mean:" << s5_ave << " std:" << std::sqrt(s5_dev) << "\n"; // NOLINT
    ss << " total             :" << best.value().get(SCORE::TOTAL);
    // clang-format on
    RCLCPP_INFO_STREAM(rclcpp::get_logger(__func__), ss.str());
  }

  auto get(const SCORE & score_type) const -> std::vector<double>
  {
    std::vector<double> ret;
    for (const auto & data : sampling.data) {
      const auto value = data.get(score_type);
      if (std::isfinite(value)) ret.push_back(value);
    }
    return ret;
  }

  ManualDrivingData manual;
  SamplingTrajectoryData sampling;

  std::shared_ptr<RouteHandler> route_handler;

  std::shared_ptr<Parameters> parameters;
};

}  // namespace autoware::behavior_analyzer

#endif  // DATA_STRUCTS_HPP_
