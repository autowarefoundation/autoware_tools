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
  SIZE
};

enum class SCORE {
  LATERAL_COMFORTABILITY = 0,
  LONGITUDINAL_COMFORTABILITY = 1,
  EFFICIENCY = 2,
  SAFETY = 3,
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

struct Parameters
{
  size_t resample_num{20};
  double time_resolution{0.5};
  double w_lat_comfortability{1.0};
  double w_lon_comfortability{1.0};
  double w_efficiency{1.0};
  double w_safety{1.0};
  double dt{1.0};
  std::vector<double> grid{};
  TargetStateParameters target_state{};
};

struct BufferBase
{
  virtual bool ready() const = 0;
  virtual void remove_old_data(const rcutils_time_point_value_t now) = 0;
};

template <class T>
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

  auto get(const rcutils_time_point_value_t now) const -> std::optional<T>
  {
    const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() > now;
    });

    if (itr == msgs.end()) {
      return std::nullopt;
    }

    return *itr;
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
  -> std::optional<SteeringReport>;

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> std::optional<TFMessage>;

struct BagData
{
  explicit BagData(const rcutils_time_point_value_t timestamp) : timestamp{timestamp}
  {
    buffers.emplace(TOPIC::TF, std::make_shared<Buffer<TFMessage>>());
    buffers.emplace(TOPIC::ODOMETRY, std::make_shared<Buffer<Odometry>>());
    buffers.emplace(TOPIC::ACCELERATION, std::make_shared<Buffer<AccelWithCovarianceStamped>>());
    buffers.emplace(TOPIC::TRAJECTORY, std::make_shared<Buffer<Trajectory>>());
    buffers.emplace(TOPIC::OBJECTS, std::make_shared<Buffer<PredictedObjects>>());
    buffers.emplace(TOPIC::STEERING, std::make_shared<Buffer<SteeringReport>>());
  }

  std::map<std::string, std::shared_ptr<BufferBase>> buffers{};

  rcutils_time_point_value_t timestamp;

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
    const std::shared_ptr<BagData> & trimmed_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag);

  void calculate();

  double longitudinal_comfortability() const;

  double lateral_comfortability() const;

  double efficiency() const;

  double safety() const;

  double total() const;

  virtual double lateral_accel(const size_t idx) const = 0;

  virtual double longitudinal_jerk(const size_t idx) const = 0;

  virtual double minimum_ttc(const size_t idx) const = 0;

  virtual double travel_distance(const size_t idx) const = 0;

  virtual bool feasible() const = 0;

  std::vector<PredictedObjects> objects_history;

  std::unordered_map<METRIC, std::vector<double>> values;
  std::unordered_map<SCORE, double> scores;

  vehicle_info_utils::VehicleInfo vehicle_info;

  std::shared_ptr<Parameters> parameters;

  std::string tag{""};
};

struct ManualDrivingData : CommonData
{
  ManualDrivingData(
    const std::shared_ptr<BagData> & trimmed_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters);

  double lateral_accel(const size_t idx) const override;

  double longitudinal_jerk(const size_t idx) const override;

  double minimum_ttc(const size_t idx) const override;

  double travel_distance(const size_t idx) const override;

  bool feasible() const override { return true; }

  std::vector<Odometry> odometry_history;
  std::vector<AccelWithCovarianceStamped> accel_history;
  std::vector<SteeringReport> steer_history;
};

struct TrajectoryData : CommonData
{
  TrajectoryData(
    const std::shared_ptr<BagData> & trimmed_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters, const std::string & tag,
    const std::vector<TrajectoryPoint> & points);

  double lateral_accel(const size_t idx) const override;

  double longitudinal_jerk(const size_t idx) const override;

  double minimum_ttc(const size_t idx) const override;

  double travel_distance(const size_t idx) const override;

  bool feasible() const override;

  std::vector<TrajectoryPoint> points;
};

struct SamplingTrajectoryData
{
  SamplingTrajectoryData(
    const std::shared_ptr<BagData> & trimmed_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters);

  auto best() const -> std::optional<TrajectoryData>
  {
    if (data.empty()) return std::nullopt;
    return data.front();
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
    const std::shared_ptr<BagData> & trimmed_data,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<Parameters> & parameters)
  : manual{ManualDrivingData(trimmed_data, vehicle_info, parameters)},
    sampling{SamplingTrajectoryData(trimmed_data, vehicle_info, parameters)}
  {
  }

  auto loss() const -> std::optional<double>
  {
    const auto best = sampling.best();
    if (!best.has_value()) {
      return std::nullopt;
    }

    if (manual.odometry_history.size() != best.value().points.size()) {
      return std::nullopt;
    }

    double mse = 0.0;
    for (size_t i = 0; i < manual.odometry_history.size(); i++) {
      const auto & p1 = manual.odometry_history.at(i).pose.pose;
      const auto & p2 = best.value().points.at(i);
      mse = (mse * i + autoware::universe_utils::calcSquaredDistance2d(p1, p2)) / (i + 1);
    }

    return mse;
  }

  ManualDrivingData manual;
  SamplingTrajectoryData sampling;
};

}  // namespace autoware::behavior_analyzer

#endif  // DATA_STRUCTS_HPP_
