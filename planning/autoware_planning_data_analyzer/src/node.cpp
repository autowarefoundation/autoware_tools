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
  previous_{nullptr},
  buffer_{static_cast<size_t>(SCORE::SIZE)}
{
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 20ms, std::bind(&BehaviorAnalyzerNode::on_timer, this));

  timer_->cancel();

  vehicle_info_ = std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo());

  pub_marker_ = create_publisher<MarkerArray>("~/marker", 1);
  pub_odometry_ = create_publisher<Odometry>(TOPIC::ODOMETRY, rclcpp::QoS(1));
  pub_objects_ = create_publisher<PredictedObjects>(TOPIC::OBJECTS, rclcpp::QoS(1));
  pub_trajectory_ = create_publisher<Trajectory>(TOPIC::TRAJECTORY, rclcpp::QoS(1));
  pub_tf_ = create_publisher<TFMessage>(TOPIC::TF, rclcpp::QoS(1));

  pub_manual_metrics_ =
    create_publisher<Float32MultiArrayStamped>("~/manual_metrics", rclcpp::QoS{1});
  pub_system_metrics_ =
    create_publisher<Float32MultiArrayStamped>("~/system_metrics", rclcpp::QoS{1});
  pub_manual_score_ = create_publisher<Float32MultiArrayStamped>("~/manual_score", rclcpp::QoS{1});
  pub_system_score_ = create_publisher<Float32MultiArrayStamped>("~/system_score", rclcpp::QoS{1});

  sub_map_ = create_subscription<LaneletMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  srv_play_ = this->create_service<SetBool>(
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

  evaluator_parameters_ = std::make_shared<EvaluatorParameters>();
  evaluator_parameters_->resample_num = declare_parameter<int>("resample_num");
  evaluator_parameters_->time_resolution = declare_parameter<double>("time_resolution");
  evaluator_parameters_->grid_seach.dt = declare_parameter<double>("grid_seach.dt");
  evaluator_parameters_->grid_seach.min = declare_parameter<double>("grid_seach.min");
  evaluator_parameters_->grid_seach.max = declare_parameter<double>("grid_seach.max");
  evaluator_parameters_->grid_seach.resolusion = declare_parameter<double>("grid_seach.resolusion");
  evaluator_parameters_->grid_seach.thread_num = declare_parameter<int>("grid_seach.thread_num");
  evaluator_parameters_->target_state.lat_positions =
    declare_parameter<std::vector<double>>("target_state.lateral_positions");
  evaluator_parameters_->target_state.lat_velocities =
    declare_parameter<std::vector<double>>("target_state.lateral_velocities");
  evaluator_parameters_->target_state.lat_accelerations =
    declare_parameter<std::vector<double>>("target_state.lateral_accelerations");
  evaluator_parameters_->target_state.lon_positions =
    declare_parameter<std::vector<double>>("target_state.longitudinal_positions");
  evaluator_parameters_->target_state.lon_velocities =
    declare_parameter<std::vector<double>>("target_state.longitudinal_velocities");
  evaluator_parameters_->target_state.lon_accelerations =
    declare_parameter<std::vector<double>>("target_state.longitudinal_accelerations");

  selector_parameters_ = std::make_shared<SelectorParameters>(evaluator_parameters_->resample_num);
  selector_parameters_->time_decay_weight.at(0) =
    declare_parameter<std::vector<double>>("time_decay_weight.s0");
  selector_parameters_->time_decay_weight.at(1) =
    declare_parameter<std::vector<double>>("time_decay_weight.s1");
  selector_parameters_->time_decay_weight.at(2) =
    declare_parameter<std::vector<double>>("time_decay_weight.s2");
  selector_parameters_->time_decay_weight.at(3) =
    declare_parameter<std::vector<double>>("time_decay_weight.s3");
  selector_parameters_->time_decay_weight.at(4) =
    declare_parameter<std::vector<double>>("time_decay_weight.s4");
  selector_parameters_->time_decay_weight.at(5) =
    declare_parameter<std::vector<double>>("time_decay_weight.s5");
  selector_parameters_->score_weight = declare_parameter<std::vector<double>>("score_weight");
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
  const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!req->data) {
    timer_->cancel();
    return;
  }

  bag_data_ = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  timer_->reset();

  RCLCPP_INFO(get_logger(), "start evaluation.");

  res->success = true;
}

void BehaviorAnalyzerNode::rewind(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  reader_.seek(0);

  bag_data_.reset();
  bag_data_ = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  res->success = true;
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

  RCLCPP_INFO(get_logger(), "update route.");
  res->success = true;
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

  double resolusion = evaluator_parameters_->grid_seach.resolusion;
  double min = evaluator_parameters_->grid_seach.min;
  double max = evaluator_parameters_->grid_seach.max;
  for (double w0 = min; w0 < max + 0.1 * resolusion; w0 += resolusion) {
    for (double w1 = min; w1 < max + 0.1 * resolusion; w1 += resolusion) {
      for (double w2 = min; w2 < max + 0.1 * resolusion; w2 += resolusion) {
        for (double w3 = min; w3 < max + 0.1 * resolusion; w3 += resolusion) {
          for (double w4 = min; w4 < max + 0.1 * resolusion; w4 += resolusion) {
            for (double w5 = min; w5 < max + 0.1 * resolusion; w5 += resolusion) {
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

  while (reader_.has_next() && rclcpp::ok()) {
    update(bag_data, evaluator_parameters_->grid_seach.dt);

    if (!bag_data->ready()) break;

    // TODO: use previous points
    const auto bag_evaluator = std::make_shared<BagEvaluator>(
      bag_data, nullptr, route_handler_, vehicle_info_, evaluator_parameters_);

    std::mutex grid_mutex;

    // TODO: set time_decay_weight
    const auto update = [&weight_grid, &grid_mutex](const auto & bag_evaluator, const auto idx) {
      const auto selector_parameters = std::make_shared<SelectorParameters>(20);
      {
        std::lock_guard<std::mutex> lock(grid_mutex);
        if (idx + 1 > weight_grid.size()) return;
        selector_parameters->score_weight = weight_grid.at(idx).weight;
      }

      const auto [loss, points] = bag_evaluator->loss(selector_parameters);

      {
        std::lock_guard<std::mutex> lock(grid_mutex);
        if (idx < weight_grid.size()) {
          weight_grid.at(idx).loss += loss;
          weight_grid.at(idx).previous_points = points;
        }
      }
    };

    size_t i = 0;
    while (rclcpp::ok()) {
      std::vector<std::thread> threads;
      for (size_t thread_id = 0; thread_id < evaluator_parameters_->grid_seach.thread_num;
           thread_id++) {
        threads.emplace_back(update, bag_evaluator, i + thread_id);
      }

      for (auto & t : threads) t.join();

      if (i + 1 >= weight_grid.size()) break;

      i += evaluator_parameters_->grid_seach.thread_num;
    }

    std::cout << "IDX:" << i << " GRID:" << weight_grid.size() << std::endl;

    show_best_result();
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    "finish weight grid search. processing time:" << stop_watch.toc("total_time") << "[ms]");

  res->success = true;
}

void BehaviorAnalyzerNode::analyze(const std::shared_ptr<BagData> & bag_data) const
{
  if (!bag_data->ready()) return;

  const auto bag_evaluator = std::make_shared<BagEvaluator>(
    bag_data, previous_, route_handler_, vehicle_info_, evaluator_parameters_);

  const auto opt_tf = std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
                        ->get(bag_data->timestamp);
  if (opt_tf) {
    pub_tf_->publish(*opt_tf);
  }

  const auto opt_objects =
    std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS))
      ->get(bag_data->timestamp);
  if (opt_objects) {
    pub_objects_->publish(*opt_objects);
  }

  const auto opt_trajectory =
    std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
      ->get(bag_data->timestamp);
  if (opt_trajectory) {
    pub_trajectory_->publish(*opt_trajectory);
  }

  // metrics(bag_evaluator);

  // score(bag_evaluator);

  visualize(bag_evaluator);
}

// void BehaviorAnalyzerNode::metrics(const std::shared_ptr<BagEvaluator> & bag_evaluator) const
// {
//   {
//     Float32MultiArrayStamped msg{};

//     msg.stamp = now();
//     msg.data.resize(static_cast<size_t>(METRIC::SIZE) * parameters_->resample_num);

//     const auto set_metrics = [&msg, this](const auto & data, const auto metric_type) {
//       const auto offset = static_cast<size_t>(metric_type) * parameters_->resample_num;
//       const auto metric = data.values.at(static_cast<size_t>(metric_type));
//       std::copy(metric.begin(), metric.end(), msg.data.begin() + offset);
//     };

//     set_metrics(bag_evaluator->manual, METRIC::LATERAL_ACCEL);
//     set_metrics(bag_evaluator->manual, METRIC::LONGITUDINAL_JERK);
//     set_metrics(bag_evaluator->manual, METRIC::TRAVEL_DISTANCE);
//     set_metrics(bag_evaluator->manual, METRIC::MINIMUM_TTC);
//     set_metrics(bag_evaluator->manual, METRIC::LATERAL_DEVIATION);

//     pub_manual_metrics_->publish(msg);
//   }

//   const auto autoware_trajectory = bag_evaluator->sampling.autoware();
//   if (autoware_trajectory.has_value()) {
//     Float32MultiArrayStamped msg{};

//     msg.stamp = now();
//     msg.data.resize(static_cast<size_t>(METRIC::SIZE) * parameters_->resample_num);

//     const auto set_metrics = [&msg, this](const auto & data, const auto metric_type) {
//       const auto offset = static_cast<size_t>(metric_type) * parameters_->resample_num;
//       const auto metric = data.values.at(static_cast<size_t>(metric_type));
//       std::copy(metric.begin(), metric.end(), msg.data.begin() + offset);
//     };

//     set_metrics(autoware_trajectory.value(), METRIC::LATERAL_ACCEL);
//     set_metrics(autoware_trajectory.value(), METRIC::LONGITUDINAL_JERK);
//     set_metrics(autoware_trajectory.value(), METRIC::TRAVEL_DISTANCE);
//     set_metrics(autoware_trajectory.value(), METRIC::MINIMUM_TTC);
//     set_metrics(autoware_trajectory.value(), METRIC::LATERAL_DEVIATION);

//     pub_system_metrics_->publish(msg);
//   }
// }

// void BehaviorAnalyzerNode::score(const std::shared_ptr<BagEvaluator> & bag_evaluator) const
// {
//   {
//     Float32MultiArrayStamped msg{};

//     msg.stamp = now();
//     msg.data.resize(static_cast<size_t>(SCORE::SIZE));

//     const auto set_reward = [&msg](const auto & data, const auto score_type) {
//       msg.data.at(static_cast<size_t>(static_cast<size_t>(score_type))) =
//         static_cast<float>(data.scores.at(static_cast<size_t>(score_type)));
//     };

//     set_reward(bag_evaluator->manual, SCORE::LONGITUDINAL_COMFORTABILITY);
//     set_reward(bag_evaluator->manual, SCORE::LATERAL_COMFORTABILITY);
//     set_reward(bag_evaluator->manual, SCORE::EFFICIENCY);
//     set_reward(bag_evaluator->manual, SCORE::SAFETY);
//     set_reward(bag_evaluator->manual, SCORE::ACHIEVABILITY);

//     pub_manual_score_->publish(msg);
//   }

//   const auto autoware_trajectory = bag_evaluator->sampling.autoware();
//   if (autoware_trajectory.has_value()) {
//     Float32MultiArrayStamped msg{};

//     msg.stamp = now();
//     msg.data.resize(static_cast<size_t>(SCORE::SIZE));

//     const auto set_reward = [&msg](const auto & data, const auto score_type) {
//       msg.data.at(static_cast<size_t>(static_cast<size_t>(score_type))) =
//         static_cast<float>(data.scores.at(static_cast<size_t>(score_type)));
//     };

//     set_reward(autoware_trajectory.value(), SCORE::LONGITUDINAL_COMFORTABILITY);
//     set_reward(autoware_trajectory.value(), SCORE::LATERAL_COMFORTABILITY);
//     set_reward(autoware_trajectory.value(), SCORE::EFFICIENCY);
//     set_reward(autoware_trajectory.value(), SCORE::SAFETY);
//     set_reward(autoware_trajectory.value(), SCORE::ACHIEVABILITY);

//     pub_system_score_->publish(msg);
//   }
// }

void BehaviorAnalyzerNode::visualize(const std::shared_ptr<BagEvaluator> & bag_evaluator) const
{
  MarkerArray msg;

  const auto ground_truth = bag_evaluator->get("ground_truth");
  if (ground_truth != nullptr) {
    for (size_t i = 0; i < ground_truth->points()->size(); ++i) {
      Marker marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ground_truth", i, Marker::ARROW,
        createMarkerScale(0.7, 0.3, 0.3), createMarkerColor(1.0, 0.0, 0.0, 0.999));
      marker.pose = ground_truth->points()->at(i).pose;
      msg.markers.push_back(marker);
    }
  }

  const auto best_data = bag_evaluator->best(selector_parameters_);

  if (best_data != nullptr) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "best_score", 0L, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & point : *best_data->points()) {
      marker.points.push_back(point.pose.position);
    }
    msg.markers.push_back(marker);
    previous_ = best_data->points();
  } else {
    previous_ = nullptr;
  }

  const auto results = bag_evaluator->results();
  for (size_t i = 0; i < results.size(); ++i) {
    msg.markers.push_back(utils::to_marker(results.at(i), SCORE::LATERAL_COMFORTABILITY, i));
    msg.markers.push_back(utils::to_marker(results.at(i), SCORE::LONGITUDINAL_COMFORTABILITY, i));
    msg.markers.push_back(utils::to_marker(results.at(i), SCORE::EFFICIENCY, i));
    msg.markers.push_back(utils::to_marker(results.at(i), SCORE::SAFETY, i));
    msg.markers.push_back(utils::to_marker(results.at(i), SCORE::ACHIEVABILITY, i));
    msg.markers.push_back(utils::to_marker(results.at(i), SCORE::CONSISTENCY, i));
  }

  {
    autoware::universe_utils::appendMarkerArray(
      lanelet::visualization::laneletsAsTriangleMarkerArray(
        "preferred_lanes", route_handler_->getPreferredLanelets(),
        createMarkerColor(0.16, 1.0, 0.69, 0.2)),
      &msg);
  }

  pub_marker_->publish(msg);

  bag_evaluator->show();
}

void BehaviorAnalyzerNode::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  update(bag_data_, 0.1);
  analyze(bag_data_);
}
}  // namespace autoware::behavior_analyzer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_analyzer::BehaviorAnalyzerNode)
