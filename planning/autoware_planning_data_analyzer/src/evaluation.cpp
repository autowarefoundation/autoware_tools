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

#include "evaluation.hpp"

#include "bag_handler.hpp"
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

DataInterface::DataInterface(
  const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<TrajectoryPoints> & previous,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters, const std::string & tag,
  const std::shared_ptr<TrajectoryPoints> & points)
: previous_{previous},
  vehicle_info{vehicle_info},
  route_handler{bag_data->route_handler},
  parameters{parameters},
  tag{tag},
  points_{points}
{
  objects_history.reserve(parameters->resample_num);

  const auto objects_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS));
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

void DataInterface::calculate()
{
  if (!feasible()) {
    scores.at(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY)) = 0.0;
    scores.at(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY)) = 0.0;
    scores.at(static_cast<size_t>(SCORE::EFFICIENCY)) = 0.0;
    scores.at(static_cast<size_t>(SCORE::SAFETY)) = 0.0;
    scores.at(static_cast<size_t>(SCORE::ACHIEVABILITY)) = 0.0;
    scores.at(static_cast<size_t>(SCORE::CONSISTENCY)) = 0.0;
  }

  std::vector<double> lateral_accel_values;
  std::vector<double> minimum_ttc_values;
  std::vector<double> longitudinal_jerk_values;
  std::vector<double> travel_distance_values;
  std::vector<double> lateral_deviation_values;
  std::vector<double> trajectory_deviation_values;

  for (size_t i = 0; i < parameters->resample_num - 1; i++) {
    lateral_accel_values.push_back(lateral_accel(i));
    longitudinal_jerk_values.push_back(longitudinal_jerk(i));
    minimum_ttc_values.push_back(minimum_ttc(i));
    travel_distance_values.push_back(travel_distance(i));
    lateral_deviation_values.push_back(lateral_deviation(i));
    trajectory_deviation_values.push_back(trajectory_deviation(i));
  }

  {
    lateral_accel_values.push_back(lateral_accel(parameters->resample_num - 1));
    longitudinal_jerk_values.push_back(0.0);
    minimum_ttc_values.push_back(minimum_ttc(parameters->resample_num - 1));
    travel_distance_values.push_back(travel_distance(parameters->resample_num - 1));
    lateral_deviation_values.push_back(lateral_deviation(parameters->resample_num - 1));
    trajectory_deviation_values.push_back(trajectory_deviation(parameters->resample_num - 1));
  }

  values.at(static_cast<size_t>(METRIC::LATERAL_ACCEL)) = lateral_accel_values;
  values.at(static_cast<size_t>(METRIC::LONGITUDINAL_JERK)) = longitudinal_jerk_values;
  values.at(static_cast<size_t>(METRIC::MINIMUM_TTC)) = minimum_ttc_values;
  values.at(static_cast<size_t>(METRIC::TRAVEL_DISTANCE)) = travel_distance_values;
  values.at(static_cast<size_t>(METRIC::LATERAL_DEVIATION)) = lateral_deviation_values;
  values.at(static_cast<size_t>(METRIC::TRAJECTORY_DEVIATION)) = trajectory_deviation_values;

  scores.at(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY)) = to_1d(METRIC::LATERAL_ACCEL);
  scores.at(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY)) =
    to_1d(METRIC::LONGITUDINAL_JERK);
  scores.at(static_cast<size_t>(SCORE::EFFICIENCY)) = to_1d(METRIC::TRAVEL_DISTANCE);
  scores.at(static_cast<size_t>(SCORE::SAFETY)) = to_1d(METRIC::MINIMUM_TTC);
  scores.at(static_cast<size_t>(SCORE::ACHIEVABILITY)) = to_1d(METRIC::LATERAL_DEVIATION);
  scores.at(static_cast<size_t>(SCORE::CONSISTENCY)) = to_1d(METRIC::TRAJECTORY_DEVIATION);
}

double DataInterface::lateral_accel(const size_t idx) const
{
  const auto radius = vehicle_info.wheel_base_m / std::tan(points_->at(idx).front_wheel_angle_rad);
  const auto speed = points_->at(idx).longitudinal_velocity_mps;
  return speed * speed / radius;
}

double DataInterface::longitudinal_jerk(const size_t idx) const
{
  return (points_->at(idx + 1).acceleration_mps2 - points_->at(idx).acceleration_mps2) / 0.5;
}

double DataInterface::minimum_ttc(const size_t idx) const
{
  const auto p_ego = points_->at(idx).pose;
  const auto v_ego = utils::get_velocity_in_world_coordinate(points_->at(idx));

  return utils::time_to_collision(*objects_history.at(idx), p_ego, v_ego);
}

double DataInterface::travel_distance(const size_t idx) const
{
  return autoware::motion_utils::calcSignedArcLength(*points_, 0L, idx);
}

double DataInterface::lateral_deviation(const size_t idx) const
{
  lanelet::ConstLanelet nearest{};
  if (!route_handler->getClosestPreferredLaneletWithinRoute(
        autoware::universe_utils::getPose(points_->at(idx)), &nearest)) {
    return std::numeric_limits<double>::max();
  }
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(
    {nearest}, autoware::universe_utils::getPose(points_->at(idx)));
  return arc_coordinates.distance;
}

double DataInterface::trajectory_deviation(const size_t idx) const
{
  if (previous_ == nullptr) return 0.0;

  const auto & p1 = autoware::universe_utils::getPose(points_->at(idx));
  const auto & p2 = previous_->at(idx);
  return autoware::universe_utils::calcSquaredDistance2d(p1, p2);
}

bool DataInterface::feasible() const
{
  const auto condition = [](const auto & p) { return p.longitudinal_velocity_mps >= 0.0; };
  return std::all_of(points_->begin(), points_->end(), condition);
}

bool DataInterface::ready() const
{
  if (objects_history.size() < parameters->resample_num) {
    return false;
  }

  if (points_->size() < parameters->resample_num) {
    return false;
  }

  return true;
}

void DataInterface::normalize(
  const double min, const double max, const size_t score_type, const bool flip)
{
  scores.at(score_type) = flip ? (max - scores.at(score_type)) / (max - min)
                               : (scores.at(score_type) - min) / (max - min);
}

auto DataInterface::to_1d(const METRIC & metric_type) const -> double
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  for (size_t i = 0; i < parameters->resample_num; i++) {
    score += std::pow(TIME_FACTOR, i) * std::abs(values.at(static_cast<size_t>(metric_type)).at(i));
  }

  return score;
}

double DataInterface::get(const SCORE & score_type) const
{
  return scores.at(static_cast<size_t>(score_type));
}

auto DataInterface::total(const std::vector<double> & weight) const -> double
{
  return std::inner_product(weight.begin(), weight.end(), scores.begin(), 0.0);
}

GroundTruth::GroundTruth(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<TrajectoryPoints> & prev_best_data,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters, const std::string & tag)
: DataInterface(
    bag_data, prev_best_data, vehicle_info, parameters, tag, to_points(bag_data, parameters))
{
  calculate();
}

auto GroundTruth::to_points(
  const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<Parameters> & parameters)
  -> std::shared_ptr<TrajectoryPoints>
{
  TrajectoryPoints points;

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

    const auto opt_accel =
      acceleration_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->time_resolution * i);
    if (!opt_accel) {
      break;
    }

    const auto opt_steer =
      steering_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->time_resolution * i);
    if (!opt_steer) {
      break;
    }

    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(opt_odometry->pose.pose)
                         .longitudinal_velocity_mps(opt_odometry->twist.twist.linear.x)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(opt_accel->accel.accel.linear.x)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(opt_steer->steering_tire_angle)
                         .rear_wheel_angle_rad(0.0);
    points.push_back(point);
  }

  return std::make_shared<TrajectoryPoints>(points);
}

TrajectoryData::TrajectoryData(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<TrajectoryPoints> & prev_best_data,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters, const std::string & tag,
  const std::shared_ptr<TrajectoryPoints> & points)
: DataInterface(bag_data, prev_best_data, vehicle_info, parameters, tag, points)
{
  calculate();
}

Evaluator::Evaluator(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<TrajectoryPoints> & prev_best_data,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<Parameters> & parameters)
: parameters{parameters}
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

  // frenet planner
  for (const auto & points : utils::sampling(
         *opt_trajectory, opt_odometry->pose.pose, opt_odometry->twist.twist.linear.x,
         opt_accel->accel.accel.linear.x, vehicle_info, parameters)) {
    const auto ptr = std::make_shared<TrajectoryData>(
      bag_data, prev_best_data, vehicle_info, parameters, "frenet",
      std::make_shared<TrajectoryPoints>(points));
    data_set.push_back(static_cast<std::shared_ptr<DataInterface>>(ptr));
  }

  // actual behavior
  {
    const auto ptr = std::make_shared<GroundTruth>(
      bag_data, prev_best_data, vehicle_info, parameters, "ground_truth");
    data_set.push_back(static_cast<std::shared_ptr<DataInterface>>(ptr));
  }

  normalize_scores();
}

void Evaluator::normalize_scores()
{
  const auto range = [this](const size_t idx) {
    const auto min_itr = std::min_element(
      data_set.begin(), data_set.end(),
      [&idx](const auto & a, const auto & b) { return a->scores.at(idx) < b->scores.at(idx); });
    const auto max_itr = std::max_element(
      data_set.begin(), data_set.end(),
      [&idx](const auto & a, const auto & b) { return a->scores.at(idx) < b->scores.at(idx); });

    return std::make_pair((*min_itr)->scores.at(idx), (*max_itr)->scores.at(idx));
  };

  const auto [s0_min, s0_max] = range(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY));
  const auto [s1_min, s1_max] = range(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY));
  const auto [s2_min, s2_max] = range(static_cast<size_t>(SCORE::EFFICIENCY));
  const auto [s3_min, s3_max] = range(static_cast<size_t>(SCORE::SAFETY));
  const auto [s4_min, s4_max] = range(static_cast<size_t>(SCORE::ACHIEVABILITY));
  const auto [s5_min, s5_max] = range(static_cast<size_t>(SCORE::CONSISTENCY));

  for (auto & data : data_set) {
    data->normalize(s0_min, s0_max, static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY), true);
    data->normalize(s1_min, s1_max, static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY), true);
    data->normalize(s2_min, s2_max, static_cast<size_t>(SCORE::EFFICIENCY));
    data->normalize(s3_min, s3_max, static_cast<size_t>(SCORE::SAFETY));
    data->normalize(s4_min, s4_max, static_cast<size_t>(SCORE::ACHIEVABILITY), true);
    data->normalize(s5_min, s5_max, static_cast<size_t>(SCORE::CONSISTENCY), true);
    data->scores.at(static_cast<size_t>(SCORE::TOTAL)) = data->total(parameters->weight);
  }

  const auto [total_min, total_max] = range(static_cast<size_t>(SCORE::TOTAL));

  for (auto & data : data_set) {
    data->normalize(total_min, total_max, static_cast<size_t>(SCORE::TOTAL));
  }
}

auto Evaluator::get(const std::string & tag) const -> std::shared_ptr<DataInterface>
{
  const auto itr = std::find_if(
    data_set.begin(), data_set.end(), [&tag](const auto & data) { return data->tag == tag; });

  return itr != data_set.end() ? *itr : nullptr;
}

auto Evaluator::best(const std::vector<double> & weight) const -> std::shared_ptr<DataInterface>
{
  auto sort_by_score = data_set;

  std::sort(sort_by_score.begin(), sort_by_score.end(), [&weight](const auto & a, const auto & b) {
    return a->total(weight) > b->total(weight);
  });

  const auto itr = std::remove_if(
    sort_by_score.begin(), sort_by_score.end(), [](const auto & d) { return !d->feasible(); });

  sort_by_score.erase(itr, sort_by_score.end());

  if (sort_by_score.empty()) return nullptr;

  return sort_by_score.front();
}

auto Evaluator::loss(const std::vector<double> & weight) const -> double
{
  const auto best_data = best(weight);
  if (best_data == nullptr) {
    return 0.0;
  }

  const auto ground_truth = get("ground_truth");
  const auto min_size = std::min(ground_truth->points()->size(), best_data->points()->size());

  double mse = 0.0;
  for (size_t i = 0; i < min_size; i++) {
    const auto & p1 = autoware::universe_utils::getPose(ground_truth->points()->at(i));
    const auto & p2 = autoware::universe_utils::getPose(best_data->points()->at(i));
    mse = (mse * i + autoware::universe_utils::calcSquaredDistance2d(p1, p2)) / (i + 1);
  }

  if (!std::isfinite(mse)) {
    throw std::logic_error("loss value is invalid.");
  }

  return mse;
}

void Evaluator::show()
{
  const auto best_data = best(parameters->weight);

  if (best_data == nullptr) {
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

  for (size_t i = 0; i < data_set.size(); i++) {
    const auto data = data_set.at(i);
    std::tie(s0_ave, s0_dev) = update(s0_ave, s0_dev, data->get(SCORE::LATERAL_COMFORTABILITY), i);
    std::tie(s1_ave, s1_dev) =
      update(s1_ave, s1_dev, data->get(SCORE::LONGITUDINAL_COMFORTABILITY), i);
    std::tie(s2_ave, s2_dev) = update(s2_ave, s2_dev, data->get(SCORE::EFFICIENCY), i);
    std::tie(s3_ave, s3_dev) = update(s3_ave, s3_dev, data->get(SCORE::SAFETY), i);
    std::tie(s4_ave, s4_dev) = update(s4_ave, s4_dev, data->get(SCORE::ACHIEVABILITY), i);
    std::tie(s5_ave, s5_dev) = update(s5_ave, s5_dev, data->get(SCORE::CONSISTENCY), i);
  }

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << "\n";
  // clang-format off
  ss << " tag               :" << best_data->tag << "\n";
  ss << " lat comfortability:" << best_data->get(SCORE::LATERAL_COMFORTABILITY)       << " mean:" << s0_ave << " std:" << std::sqrt(s0_dev) << "\n"; // NOLINT
  ss << " lon comfortability:" << best_data->get(SCORE::LONGITUDINAL_COMFORTABILITY)  << " mean:" << s1_ave << " std:" << std::sqrt(s1_dev) << "\n"; // NOLINT
  ss << " efficiency        :" << best_data->get(SCORE::EFFICIENCY)                   << " mean:" << s2_ave << " std:" << std::sqrt(s2_dev) << "\n"; // NOLINT
  ss << " safety            :" << best_data->get(SCORE::SAFETY)                       << " mean:" << s3_ave << " std:" << std::sqrt(s3_dev) << "\n"; // NOLINT
  ss << " achievability     :" << best_data->get(SCORE::ACHIEVABILITY)                << " mean:" << s4_ave << " std:" << std::sqrt(s4_dev) << "\n"; // NOLINT
  ss << " consistency       :" << best_data->get(SCORE::CONSISTENCY)                  << " mean:" << s5_ave << " std:" << std::sqrt(s5_dev) << "\n"; // NOLINT
  ss << " total             :" << best_data->get(SCORE::TOTAL);
  // clang-format on
  RCLCPP_INFO_STREAM(rclcpp::get_logger(__func__), ss.str());
}

auto Evaluator::get(const SCORE & score_type) const -> std::vector<double>
{
  std::vector<double> ret;
  for (const auto & data : data_set) {
    const auto value = data->get(score_type);
    if (std::isfinite(value)) ret.push_back(value);
  }
  return ret;
}
}  // namespace autoware::behavior_analyzer
