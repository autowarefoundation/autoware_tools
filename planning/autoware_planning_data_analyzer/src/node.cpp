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

#include "node.hpp"

#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "utils.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

namespace autoware::behavior_analyzer
{
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

BehaviorAnalyzerNode::BehaviorAnalyzerNode(const rclcpp::NodeOptions & node_options)
: Node("path_selector_node", node_options),
  route_handler_{std::make_shared<RouteHandler>()},
  vehicle_info_{std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())}
{
  pub_marker_ = create_publisher<MarkerArray>("~/marker", 1);

  pub_objects_ = create_publisher<PredictedObjects>(TOPIC::OBJECTS, rclcpp::QoS(1));

  pub_tf_ = create_publisher<TFMessage>(TOPIC::TF, rclcpp::QoS(1));

  sub_map_ = create_subscription<LaneletMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  srv_play_ = this->create_service<Trigger>(
    "play",
    std::bind(&BehaviorAnalyzerNode::play, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_rewind_ = this->create_service<Trigger>(
    "rewind",
    std::bind(&BehaviorAnalyzerNode::rewind, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_route_ = this->create_service<Trigger>(
    "next_route",
    std::bind(
      &BehaviorAnalyzerNode::next_route, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_weight_ = this->create_service<Trigger>(
    "weight_grid_search",
    std::bind(&BehaviorAnalyzerNode::weight, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  reader_.open(declare_parameter<std::string>("bag_path"));

  data_augument_parameters_ = std::make_shared<DataAugmentParameters>();
  data_augument_parameters_->sample_num = declare_parameter<int>("sample_num");
  data_augument_parameters_->resolution = declare_parameter<double>("resolution");
  data_augument_parameters_->target_state.lat_positions =
    declare_parameter<std::vector<double>>("target_state.lateral_positions");
  data_augument_parameters_->target_state.lat_velocities =
    declare_parameter<std::vector<double>>("target_state.lateral_velocities");
  data_augument_parameters_->target_state.lat_accelerations =
    declare_parameter<std::vector<double>>("target_state.lateral_accelerations");
  data_augument_parameters_->target_state.lon_positions =
    declare_parameter<std::vector<double>>("target_state.longitudinal_positions");
  data_augument_parameters_->target_state.lon_velocities =
    declare_parameter<std::vector<double>>("target_state.longitudinal_velocities");
  data_augument_parameters_->target_state.lon_accelerations =
    declare_parameter<std::vector<double>>("target_state.longitudinal_accelerations");

  evaluator_parameters_ =
    std::make_shared<trajectory_selector::trajectory_evaluator::EvaluatorParameters>(
      data_augument_parameters_->sample_num);
  evaluator_parameters_->time_decay_weight.at(0) =
    declare_parameter<std::vector<double>>("time_decay_weight.s0");
  evaluator_parameters_->time_decay_weight.at(1) =
    declare_parameter<std::vector<double>>("time_decay_weight.s1");
  evaluator_parameters_->time_decay_weight.at(2) =
    declare_parameter<std::vector<double>>("time_decay_weight.s2");
  evaluator_parameters_->time_decay_weight.at(3) =
    declare_parameter<std::vector<double>>("time_decay_weight.s3");
  evaluator_parameters_->time_decay_weight.at(4) =
    declare_parameter<std::vector<double>>("time_decay_weight.s4");
  evaluator_parameters_->time_decay_weight.at(5) =
    declare_parameter<std::vector<double>>("time_decay_weight.s5");
  evaluator_parameters_->score_weight = declare_parameter<std::vector<double>>("score_weight");
}

auto BehaviorAnalyzerNode::get_route() -> LaneletRoute::ConstSharedPtr
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back("/planning/mission_planning/route");
  reader_.set_filter(filter);

  if (!reader_.has_next()) {
    throw std::domain_error("not found route msg.");
  }

  rclcpp::Serialization<LaneletRoute> serializer;

  const auto deserialized_message = std::make_shared<LaneletRoute>();
  while (reader_.has_next()) {
    const auto next_data = reader_.read_next();
    if (next_data->topic_name == TOPIC::ROUTE) {
      rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      break;
    }
  }

  return deserialized_message;
}

void BehaviorAnalyzerNode::update(const std::shared_ptr<BagData> & bag_data, const double dt) const
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(TOPIC::TF);
  filter.topics.emplace_back(TOPIC::ODOMETRY);
  filter.topics.emplace_back(TOPIC::ACCELERATION);
  filter.topics.emplace_back(TOPIC::OBJECTS);
  filter.topics.emplace_back(TOPIC::STEERING);
  filter.topics.emplace_back(TOPIC::TRAJECTORY);
  reader_.set_filter(filter);

  bag_data->update(dt * 1e9);

  while (reader_.has_next()) {
    const auto next_data = reader_.read_next();
    rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);

    if (bag_data->ready()) {
      break;
    }

    if (next_data->topic_name == TOPIC::TF) {
      rclcpp::Serialization<TFMessage> serializer;
      const auto deserialized_message = std::make_shared<TFMessage>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::ODOMETRY) {
      rclcpp::Serialization<Odometry> serializer;
      const auto deserialized_message = std::make_shared<Odometry>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::ACCELERATION) {
      rclcpp::Serialization<AccelWithCovarianceStamped> serializer;
      const auto deserialized_message = std::make_shared<AccelWithCovarianceStamped>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
        bag_data->buffers.at(TOPIC::ACCELERATION))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::OBJECTS) {
      rclcpp::Serialization<PredictedObjects> serializer;
      const auto deserialized_message = std::make_shared<PredictedObjects>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::STEERING) {
      rclcpp::Serialization<SteeringReport> serializer;
      const auto deserialized_message = std::make_shared<SteeringReport>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers.at(TOPIC::STEERING))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::TRAJECTORY) {
      rclcpp::Serialization<Trajectory> serializer;
      const auto deserialized_message = std::make_shared<Trajectory>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
        ->append(*deserialized_message);
    }
  }
}

void BehaviorAnalyzerNode::play(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  const auto time_step =
    autoware::universe_utils::getOrDeclareParameter<double>(*this, "play.time_step");

  RCLCPP_INFO(get_logger(), "rosbag play now...");

  std::shared_ptr<TrajectoryPoints> previous_points{nullptr};

  while (reader_.has_next() && rclcpp::ok()) {
    update(bag_data, time_step);

    const auto bag_evaluator = std::make_shared<BagEvaluator>(
      bag_data, route_handler_, vehicle_info_, data_augument_parameters_);

    bag_evaluator->setup(previous_points);

    const auto best_data = bag_evaluator->best(evaluator_parameters_);

    previous_points = best_data == nullptr ? nullptr : best_data->points();

    pub_tf_->publish(*bag_evaluator->tf());

    pub_objects_->publish(*bag_evaluator->objects());

    pub_marker_->publish(*bag_evaluator->marker());

    bag_evaluator->show();
  }

  res->success = true;

  RCLCPP_INFO(get_logger(), "finish.");
}

void BehaviorAnalyzerNode::rewind(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  reader_.seek(0);

  res->success = true;

  RCLCPP_INFO(get_logger(), "rewind rosbag.");
}

void BehaviorAnalyzerNode::next_route(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  route_handler_->setRoute(*get_route());

  MarkerArray msg;

  autoware::universe_utils::appendMarkerArray(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "preferred_lanes", route_handler_->getPreferredLanelets(),
      createMarkerColor(0.16, 1.0, 0.69, 0.2)),
    &msg);

  pub_marker_->publish(msg);

  res->success = true;

  RCLCPP_INFO(get_logger(), "update route.");
}

void BehaviorAnalyzerNode::weight(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "start weight grid seach.");

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  stop_watch.tic("total_time");

  reader_.seek(0);
  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  std::vector<Result> weight_grid;

  const auto resolution =
    autoware::universe_utils::getOrDeclareParameter<double>(*this, "grid_seach.grid_step");
  const auto min = autoware::universe_utils::getOrDeclareParameter<double>(*this, "grid_seach.min");
  const auto max = autoware::universe_utils::getOrDeclareParameter<double>(*this, "grid_seach.max");
  for (double w0 = min; w0 < max + 0.1 * resolution; w0 += resolution) {
    for (double w1 = min; w1 < max + 0.1 * resolution; w1 += resolution) {
      for (double w2 = min; w2 < max + 0.1 * resolution; w2 += resolution) {
        for (double w3 = min; w3 < max + 0.1 * resolution; w3 += resolution) {
          for (double w4 = min; w4 < max + 0.1 * resolution; w4 += resolution) {
            for (double w5 = min; w5 < max + 0.1 * resolution; w5 += resolution) {
              weight_grid.emplace_back(w0, w1, w2, w3, w4, w5);
            }
          }
        }
      }
    }
  }

  const auto show_best_result = [this, &weight_grid]() {
    auto sort_by_loss = weight_grid;
    std::sort(sort_by_loss.begin(), sort_by_loss.end(), [](const auto & a, const auto & b) {
      return a.loss < b.loss;
    });

    const auto best = sort_by_loss.front();

    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < best.weight.size(); i++) {
      ss << " [w" << i << "]:" << best.weight.at(i);
    }
    ss << " [loss]:" << best.loss << std::endl;
    RCLCPP_INFO_STREAM(get_logger(), ss.str());
  };

  const size_t thread_num =
    autoware::universe_utils::getOrDeclareParameter<int>(*this, "grid_seach.thread_num");
  const auto time_step =
    autoware::universe_utils::getOrDeclareParameter<double>(*this, "grid_seach.time_step");

  // start grid search
  while (reader_.has_next() && rclcpp::ok()) {
    stop_watch.tic("one_step");
    update(bag_data, time_step);

    if (!bag_data->ready()) break;

    const auto bag_evaluator = std::make_shared<BagEvaluator>(
      bag_data, route_handler_, vehicle_info_, data_augument_parameters_);

    std::mutex g_mutex;
    std::mutex e_mutex;

    const auto update = [&bag_evaluator, &weight_grid, &g_mutex, &e_mutex](const auto idx) {
      const auto selector_parameters =
        std::make_shared<trajectory_selector::trajectory_evaluator::EvaluatorParameters>(20);

      double loss = 0.0;

      std::shared_ptr<TrajectoryPoints> previous_points;
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (idx + 1 > weight_grid.size()) return;
        selector_parameters->score_weight = weight_grid.at(idx).weight;
        selector_parameters->time_decay_weight = std::vector<std::vector<double>>(
          static_cast<size_t>(METRIC::SIZE),
          {1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13});
        previous_points = weight_grid.at(idx).previous_points;
      }

      std::shared_ptr<TrajectoryPoints> selected_points;
      {
        std::lock_guard<std::mutex> lock(e_mutex);
        bag_evaluator->setup(previous_points);
        std::tie(loss, selected_points) = bag_evaluator->loss(selector_parameters);
      }

      {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (idx < weight_grid.size()) {
          weight_grid.at(idx).loss += loss;
          weight_grid.at(idx).previous_points = selected_points;
        }
      }
    };

    size_t i = 0;
    while (rclcpp::ok()) {
      std::vector<std::thread> threads;
      for (size_t thread_id = 0; thread_id < thread_num; thread_id++) {
        threads.emplace_back(update, i + thread_id);
      }

      for (auto & t : threads) t.join();

      if (i + 1 >= weight_grid.size()) break;

      i += thread_num;
    }

    show_best_result();

    RCLCPP_INFO_STREAM(
      get_logger(), "it took " << stop_watch.toc("one_step") << "[ms] to search grid for "
                               << time_step << "[s] bag.");
  }

  res->success = true;

  RCLCPP_INFO_STREAM(
    get_logger(),
    "finish weight grid search. processing time:" << stop_watch.toc("total_time") << "[ms]");
}
}  // namespace autoware::behavior_analyzer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_analyzer::BehaviorAnalyzerNode)
