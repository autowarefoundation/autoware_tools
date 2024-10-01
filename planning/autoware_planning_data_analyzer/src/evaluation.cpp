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

#include <autoware/universe_utils/ros/marker_helper.hpp>
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
auto BagEvaluator::objects(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<DataAugmentParameters> & parameters) const
  -> std::shared_ptr<PredictedObjects>
{
  const auto objects_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS));

  const auto current_objects = objects_buffer_ptr->get(bag_data->timestamp);

  // remove predicted_paths.
  std::for_each(
    current_objects->objects.begin(), current_objects->objects.end(), [](auto & object) {
      object.kinematics.predicted_paths.clear();
      object.kinematics.predicted_paths.push_back(PredictedPath{});
    });

  // overwrite predicted_paths by future data.
  for (size_t i = 0; i < parameters->sample_num; i++) {
    const auto objects_ptr =
      objects_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!objects_ptr) {
      break;
    }

    for (const auto & a : objects_ptr->objects) {
      const auto itr = std::find_if(
        current_objects->objects.begin(), current_objects->objects.end(),
        [&a](const auto & b) { return a.object_id == b.object_id; });
      if (itr == current_objects->objects.end()) continue;

      itr->kinematics.predicted_paths.at(0).path.push_back(
        a.kinematics.initial_pose_with_covariance.pose);
    }
  }

  return current_objects;
}

auto BagEvaluator::ground_truth(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<DataAugmentParameters> & parameters) const
  -> std::shared_ptr<TrajectoryPoints>
{
  TrajectoryPoints points;

  const auto odometry_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY));

  const auto acceleration_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
      bag_data->buffers.at(TOPIC::ACCELERATION));

  const auto steering_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers.at(TOPIC::STEERING));

  for (size_t i = 0; i < parameters->sample_num; i++) {
    const auto odometry_ptr =
      odometry_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!odometry_ptr) {
      throw std::logic_error("data is not enough.");
    }

    const auto accel_ptr =
      acceleration_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!accel_ptr) {
      throw std::logic_error("data is not enough.");
    }

    const auto opt_steer =
      steering_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!opt_steer) {
      throw std::logic_error("data is not enough.");
    }

    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(odometry_ptr->pose.pose)
                         .longitudinal_velocity_mps(odometry_ptr->twist.twist.linear.x)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(accel_ptr->accel.accel.linear.x)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(opt_steer->steering_tire_angle)
                         .rear_wheel_angle_rad(0.0);
    points.push_back(point);
  }

  return std::make_shared<TrajectoryPoints>(points);
}

auto BagEvaluator::augment_data(
  const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<VehicleInfo> & vehicle_info,
  const std::shared_ptr<DataAugmentParameters> & parameters) const
  -> std::vector<std::shared_ptr<TrajectoryPoints>>
{
  std::vector<std::shared_ptr<TrajectoryPoints>> augment_data;

  const auto odometry_ptr =
    std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY))
      ->get(bag_data->timestamp);
  if (!odometry_ptr) {
    throw std::logic_error("data is not enough.");
  }

  const auto accel_ptr = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
                           bag_data->buffers.at(TOPIC::ACCELERATION))
                           ->get(bag_data->timestamp);
  if (!accel_ptr) {
    throw std::logic_error("data is not enough.");
  }

  const auto trajectory_ptr =
    std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
      ->get(bag_data->timestamp);
  if (!trajectory_ptr) {
    throw std::logic_error("data is not enough.");
  }

  for (const auto & points : utils::augment(
         *trajectory_ptr, odometry_ptr->pose.pose, odometry_ptr->twist.twist.linear.x,
         accel_ptr->accel.accel.linear.x, vehicle_info, parameters)) {
    augment_data.push_back(std::make_shared<TrajectoryPoints>(points));
  }

  return augment_data;
}

void BagEvaluator::setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  {
    const auto core_data = std::make_shared<trajectory_selector::trajectory_evaluator::CoreData>(
      ground_truth_, previous_points, objects_, odometry_, "ground_truth");

    add(core_data);
  }

  for (const auto & points : augment_data_) {
    const auto core_data = std::make_shared<trajectory_selector::trajectory_evaluator::CoreData>(
      points, previous_points, objects_, odometry_, "candidates");

    add(core_data);
  }
}

auto BagEvaluator::loss(
  const std::shared_ptr<trajectory_selector::trajectory_evaluator::EvaluatorParameters> &
    parameters) -> std::pair<double, std::shared_ptr<TrajectoryPoints>>
{
  const auto best_data = best(parameters, "ground_truth");

  if (best_data == nullptr) {
    return std::make_pair(0.0, nullptr);
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

  return std::make_pair(mse, best_data->points());
}

auto BagEvaluator::marker() const -> std::shared_ptr<MarkerArray>
{
  using autoware::universe_utils::createDefaultMarker;
  using autoware::universe_utils::createMarkerColor;
  using autoware::universe_utils::createMarkerScale;

  MarkerArray msg;

  const auto ground_truth = get("ground_truth");
  if (ground_truth != nullptr) {
    for (size_t i = 0; i < ground_truth->points()->size(); ++i) {
      Marker marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ground_truth", i, Marker::ARROW,
        createMarkerScale(0.7, 0.3, 0.3), createMarkerColor(1.0, 0.0, 0.0, 0.999));
      marker.pose = ground_truth->points()->at(i).pose;
      msg.markers.push_back(marker);
    }
  }

  const auto best_data = best();
  if (best_data != nullptr) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "best_score", 0L, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & point : *best_data->points()) {
      marker.points.push_back(point.pose.position);
    }
    msg.markers.push_back(marker);
  }

  for (size_t i = 0; i < results().size(); ++i) {
    msg.markers.push_back(utils::to_marker(results().at(i), SCORE::LATERAL_COMFORTABILITY, i));
    msg.markers.push_back(utils::to_marker(results().at(i), SCORE::LONGITUDINAL_COMFORTABILITY, i));
    msg.markers.push_back(utils::to_marker(results().at(i), SCORE::EFFICIENCY, i));
    msg.markers.push_back(utils::to_marker(results().at(i), SCORE::SAFETY, i));
    msg.markers.push_back(utils::to_marker(results().at(i), SCORE::ACHIEVABILITY, i));
    msg.markers.push_back(utils::to_marker(results().at(i), SCORE::CONSISTENCY, i));
  }

  return std::make_shared<MarkerArray>(msg);
}
}  // namespace autoware::behavior_analyzer
