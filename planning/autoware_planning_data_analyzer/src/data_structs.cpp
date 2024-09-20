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

#include "data_structs.hpp"

#include "utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_analyzer
{

std::string TOPIC::TF = "/tf";                                             // NOLINT
std::string TOPIC::ODOMETRY = "/localization/kinematic_state";             // NOLINT
std::string TOPIC::ACCELERATION = "/localization/acceleration";            // NOLINT
std::string TOPIC::OBJECTS = "/perception/object_recognition/objects";     // NOLINT
std::string TOPIC::TRAJECTORY = "/planning/scenario_planning/trajectory";  // NOLINT
std::string TOPIC::STEERING = "/vehicle/status/steering_status";           // NOLINT
std::string TOPIC::ROUTE = "/planning/mission_planning/route";             // NOLINT

template <>
bool Buffer<SteeringReport>::ready() const
{
  if (msgs.empty()) {
    return false;
  }

  return rclcpp::Time(msgs.back().stamp).nanoseconds() -
           rclcpp::Time(msgs.front().stamp).nanoseconds() >
         BUFFER_TIME;
}

template <>
bool Buffer<TFMessage>::ready() const
{
  if (msgs.empty()) {
    return false;
  }

  if (msgs.front().transforms.empty()) {
    return false;
  }

  if (msgs.back().transforms.empty()) {
    return false;
  }

  return rclcpp::Time(msgs.back().transforms.front().header.stamp).nanoseconds() -
           rclcpp::Time(msgs.front().transforms.front().header.stamp).nanoseconds() >
         BUFFER_TIME;
}

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now)
{
  if (msgs.empty()) {
    return;
  }

  const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
    return rclcpp::Time(msg.stamp).nanoseconds() < now;
  });
  msgs.erase(itr, msgs.end());
}

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now)
{
  if (msgs.empty()) {
    return;
  }

  const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
    return rclcpp::Time(msg.transforms.front().header.stamp).nanoseconds() < now;
  });
  msgs.erase(itr, msgs.end());
}

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const
  -> SteeringReport::SharedPtr
{
  const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
    return rclcpp::Time(msg.stamp).nanoseconds() > now;
  });

  if (itr == msgs.end()) {
    return nullptr;
  }

  return std::make_shared<SteeringReport>(*itr);
}

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr
{
  const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
    return rclcpp::Time(msg.transforms.front().header.stamp).nanoseconds() > now;
  });

  if (itr == msgs.end()) {
    return nullptr;
  }

  return std::make_shared<TFMessage>(*itr);
}

CommonData::CommonData(
  const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters, const std::string & tag)
: vehicle_info{vehicle_info},
  route_handler{bag_data->route_handler},
  parameters{parameters},
  tag{tag}
{
  objects_history.reserve(parameters->resample_num);

  const auto objects_buffer_ptr = std::dynamic_pointer_cast<Buffer<PredictedObjects>>(
    bag_data->buffers.at("/perception/object_recognition/objects"));
  for (size_t i = 0; i < parameters->resample_num; i++) {
    const auto opt_objects =
      objects_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->time_resolution * i);
    if (!opt_objects) {
      break;
    }
    objects_history.push_back(opt_objects);
  }

  values.resize(static_cast<size_t>(METRIC::SIZE));
  scores.resize(static_cast<size_t>(SCORE::SIZE));
}

void CommonData::calculate()
{
  std::vector<double> lateral_accel_values;
  std::vector<double> minimum_ttc_values;
  std::vector<double> longitudinal_jerk_values;
  std::vector<double> travel_distance_values;
  std::vector<double> lateral_deviation_values;

  for (size_t i = 0; i < parameters->resample_num - 1; i++) {
    lateral_accel_values.push_back(lateral_accel(i));
    longitudinal_jerk_values.push_back(longitudinal_jerk(i));
    minimum_ttc_values.push_back(minimum_ttc(i));
    travel_distance_values.push_back(travel_distance(i));
    lateral_deviation_values.push_back(lateral_deviation(i));
  }

  {
    lateral_accel_values.push_back(lateral_accel(parameters->resample_num - 1));
    longitudinal_jerk_values.push_back(0.0);
    minimum_ttc_values.push_back(minimum_ttc(parameters->resample_num - 1));
    travel_distance_values.push_back(travel_distance(parameters->resample_num - 1));
    lateral_deviation_values.push_back(lateral_deviation(parameters->resample_num - 1));
  }

  values.at(static_cast<size_t>(METRIC::LATERAL_ACCEL)) = lateral_accel_values;
  values.at(static_cast<size_t>(METRIC::LONGITUDINAL_JERK)) = longitudinal_jerk_values;
  values.at(static_cast<size_t>(METRIC::MINIMUM_TTC)) = minimum_ttc_values;
  values.at(static_cast<size_t>(METRIC::TRAVEL_DISTANCE)) = travel_distance_values;
  values.at(static_cast<size_t>(METRIC::LATERAL_DEVIATION)) = lateral_deviation_values;

  scores.at(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY)) = lateral_comfortability();
  scores.at(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY)) =
    longitudinal_comfortability();
  scores.at(static_cast<size_t>(SCORE::EFFICIENCY)) = efficiency();
  scores.at(static_cast<size_t>(SCORE::SAFETY)) = safety();
  scores.at(static_cast<size_t>(SCORE::ACHIEVABILITY)) = achievability();
}

double CommonData::longitudinal_comfortability() const
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 0.5;
  const auto normalize = [&min, &max](const double value) {
    return (max - std::clamp(value, min, max)) / (max - min);
  };

  for (size_t i = 0; i < parameters->resample_num; i++) {
    score += normalize(
      std::pow(TIME_FACTOR, i) *
      std::abs(values.at(static_cast<size_t>(METRIC::LONGITUDINAL_JERK)).at(i)));
  }

  return score / parameters->resample_num;
}

double CommonData::lateral_comfortability() const
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 0.5;
  const auto normalize = [&min, &max](const double value) {
    return (max - std::clamp(value, min, max)) / (max - min);
  };

  for (size_t i = 0; i < parameters->resample_num; i++) {
    score += normalize(
      std::pow(TIME_FACTOR, i) *
      std::abs(values.at(static_cast<size_t>(METRIC::LATERAL_ACCEL)).at(i)));
  }

  return score / parameters->resample_num;
}

double CommonData::efficiency() const
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 20.0;
  const auto normalize = [&min, &max](const double value) {
    return std::clamp(value, min, max) / (max - min);
  };

  for (size_t i = 0; i < parameters->resample_num; i++) {
    score += normalize(
      std::pow(TIME_FACTOR, i) * values.at(static_cast<size_t>(METRIC::TRAVEL_DISTANCE)).at(i) /
      0.5);
  }

  return score / parameters->resample_num;
}

double CommonData::safety() const
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 5.0;
  const auto normalize = [&min, &max](const double value) {
    return std::clamp(value, min, max) / (max - min);
  };

  for (size_t i = 0; i < parameters->resample_num; i++) {
    score += normalize(
      std::pow(TIME_FACTOR, i) * values.at(static_cast<size_t>(METRIC::MINIMUM_TTC)).at(i));
  }

  return score / parameters->resample_num;
}

double CommonData::achievability() const
{
  constexpr double TIME_FACTOR = 1.0;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 2.0;
  const auto normalize = [&min, &max](const double value) {
    return (max - std::clamp(value, min, max)) / (max - min);
  };

  for (size_t i = 0; i < parameters->resample_num; i++) {
    score += normalize(
      std::pow(TIME_FACTOR, i) *
      std::abs(values.at(static_cast<size_t>(METRIC::LATERAL_DEVIATION)).at(i)));
  }

  return score / parameters->resample_num;
}

double CommonData::total(
  const double w0, const double w1, const double w2, const double w3, const double w4) const
{
  return w0 * scores.at(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY)) +
         w1 * scores.at(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY)) +
         w2 * scores.at(static_cast<size_t>(SCORE::EFFICIENCY)) +
         w3 * scores.at(static_cast<size_t>(SCORE::SAFETY)) +
         w4 * scores.at(static_cast<size_t>(SCORE::ACHIEVABILITY));
}

ManualDrivingData::ManualDrivingData(
  const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters)
: CommonData(bag_data, vehicle_info, parameters, "manual")
{
  odometry_history.reserve(parameters->resample_num);
  accel_history.reserve(parameters->resample_num);
  steer_history.reserve(parameters->resample_num);

  const auto odometry_buffer_ptr = std::dynamic_pointer_cast<Buffer<Odometry>>(
    bag_data->buffers.at("/localization/kinematic_state"));
  const auto acceleration_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
      bag_data->buffers.at("/localization/acceleration"));
  const auto steering_buffer_ptr = std::dynamic_pointer_cast<Buffer<SteeringReport>>(
    bag_data->buffers.at("/vehicle/status/steering_status"));

  for (size_t i = 0; i < parameters->resample_num; i++) {
    const auto opt_odometry =
      odometry_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->time_resolution * i);
    if (!opt_odometry) {
      break;
    }
    odometry_history.push_back(opt_odometry);

    const auto opt_accel =
      acceleration_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->time_resolution * i);
    if (!opt_accel) {
      break;
    }
    accel_history.push_back(opt_accel);

    const auto opt_steer =
      steering_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->time_resolution * i);
    if (!opt_steer) {
      break;
    }
    steer_history.push_back(opt_steer);
  }

  calculate();
}

double ManualDrivingData::lateral_accel(const size_t idx) const
{
  const auto radius =
    vehicle_info.wheel_base_m / std::tan(steer_history.at(idx)->steering_tire_angle);
  const auto speed = odometry_history.at(idx)->twist.twist.linear.x;
  return speed * speed / radius;
}

double ManualDrivingData::longitudinal_jerk(const size_t idx) const
{
  const double dt = rclcpp::Time(accel_history.at(idx + 1)->header.stamp).nanoseconds() -
                    rclcpp::Time(accel_history.at(idx)->header.stamp).nanoseconds();

  return 1e9 *
         (accel_history.at(idx + 1)->accel.accel.linear.x -
          accel_history.at(idx)->accel.accel.linear.x) /
         dt;
}

double ManualDrivingData::minimum_ttc(const size_t idx) const
{
  const auto p_ego = odometry_history.at(idx)->pose.pose;
  const auto v_ego = utils::get_velocity_in_world_coordinate(*odometry_history.at(idx));

  return utils::time_to_collision(*objects_history.at(idx), p_ego, v_ego);
}

double ManualDrivingData::travel_distance(const size_t idx) const
{
  double distance = 0.0;
  for (size_t i = 0L; i < idx; i++) {
    distance += autoware::universe_utils::calcDistance3d(
      odometry_history.at(i + 1)->pose.pose, odometry_history.at(i)->pose.pose);
  }
  return distance;
}

double ManualDrivingData::lateral_deviation(const size_t idx) const
{
  lanelet::ConstLanelet nearest{};
  if (!route_handler->getClosestPreferredLaneletWithinRoute(
        odometry_history.at(idx)->pose.pose, &nearest)) {
    return std::numeric_limits<double>::max();
  }
  const auto arc_coordinates =
    lanelet::utils::getArcCoordinates({nearest}, odometry_history.at(idx)->pose.pose);
  return arc_coordinates.distance;
}

bool ManualDrivingData::ready() const
{
  if (objects_history.size() < parameters->resample_num) {
    return false;
  }

  if (odometry_history.size() < parameters->resample_num) {
    return false;
  }

  if (accel_history.size() < parameters->resample_num) {
    return false;
  }

  if (steer_history.size() < parameters->resample_num) {
    return false;
  }

  return true;
}

TrajectoryData::TrajectoryData(
  const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters, const std::string & tag,
  const std::vector<TrajectoryPoint> & points)
: CommonData(bag_data, vehicle_info, parameters, tag), points{points}
{
  calculate();
}

double TrajectoryData::lateral_accel(const size_t idx) const
{
  const auto radius = vehicle_info.wheel_base_m / std::tan(points.at(idx).front_wheel_angle_rad);
  const auto speed = points.at(idx).longitudinal_velocity_mps;
  return speed * speed / radius;
}

double TrajectoryData::longitudinal_jerk(const size_t idx) const
{
  return (points.at(idx + 1).acceleration_mps2 - points.at(idx).acceleration_mps2) / 0.5;
}

double TrajectoryData::minimum_ttc(const size_t idx) const
{
  const auto p_ego = points.at(idx).pose;
  const auto v_ego = utils::get_velocity_in_world_coordinate(points.at(idx));

  return utils::time_to_collision(*objects_history.at(idx), p_ego, v_ego);
}

double TrajectoryData::travel_distance(const size_t idx) const
{
  return autoware::motion_utils::calcSignedArcLength(points, 0L, idx);
}

double TrajectoryData::lateral_deviation(const size_t idx) const
{
  lanelet::ConstLanelet nearest{};
  if (!route_handler->getClosestPreferredLaneletWithinRoute(
        autoware::universe_utils::getPose(points.at(idx)), &nearest)) {
    return std::numeric_limits<double>::max();
  }
  const auto arc_coordinates =
    lanelet::utils::getArcCoordinates({nearest}, autoware::universe_utils::getPose(points.at(idx)));
  return arc_coordinates.distance;
}

bool TrajectoryData::feasible() const
{
  const auto condition = [](const auto & p) { return p.longitudinal_velocity_mps > -1e-3; };
  return std::all_of(points.begin(), points.end(), condition);
}

bool TrajectoryData::ready() const
{
  if (objects_history.size() < parameters->resample_num) {
    return false;
  }

  if (points.size() < parameters->resample_num) {
    return false;
  }

  return true;
}

SamplingTrajectoryData::SamplingTrajectoryData(
  const std::shared_ptr<BagData> & bag_data, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters)
{
  const auto opt_odometry = std::dynamic_pointer_cast<Buffer<Odometry>>(
                              bag_data->buffers.at("/localization/kinematic_state"))
                              ->get(bag_data->timestamp);
  if (!opt_odometry) {
    throw std::logic_error("data is not enough.");
  }

  const auto opt_accel = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
                           bag_data->buffers.at("/localization/acceleration"))
                           ->get(bag_data->timestamp);
  if (!opt_accel) {
    throw std::logic_error("data is not enough.");
  }

  const auto opt_trajectory = std::dynamic_pointer_cast<Buffer<Trajectory>>(
                                bag_data->buffers.at("/planning/scenario_planning/trajectory"))
                                ->get(bag_data->timestamp);
  if (!opt_trajectory) {
    throw std::logic_error("data is not enough.");
  }
  data.emplace_back(
    bag_data, vehicle_info, parameters, "autoware",
    utils::resampling(
      *opt_trajectory, opt_odometry->pose.pose, parameters->resample_num,
      parameters->time_resolution));

  for (const auto & sample : utils::sampling(
         *opt_trajectory, opt_odometry->pose.pose, opt_odometry->twist.twist.linear.x,
         opt_accel->accel.accel.linear.x, vehicle_info, parameters)) {
    data.emplace_back(bag_data, vehicle_info, parameters, "frenet", sample);
  }

  std::vector<TrajectoryPoint> stop_points(parameters->resample_num);
  for (auto & stop_point : stop_points) {
    stop_point.pose = opt_odometry->pose.pose;
  }
  data.emplace_back(bag_data, vehicle_info, parameters, "stop", stop_points);
}
}  // namespace autoware::behavior_analyzer
