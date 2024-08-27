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

#include <autoware/universe_utils/ros/marker_helper.hpp>

namespace autoware::behavior_analyzer
{
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

BehaviorAnalyzerNode::BehaviorAnalyzerNode(const rclcpp::NodeOptions & node_options)
: Node("path_selector_node", node_options)
{
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&BehaviorAnalyzerNode::on_timer, this));

  timer_->cancel();

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

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

  srv_play_ = this->create_service<SetBool>(
    "play",
    std::bind(&BehaviorAnalyzerNode::play, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_rewind_ = this->create_service<Trigger>(
    "rewind",
    std::bind(&BehaviorAnalyzerNode::rewind, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_weight_ = this->create_service<Trigger>(
    "weight_grid_search",
    std::bind(&BehaviorAnalyzerNode::weight, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  reader_.open(declare_parameter<std::string>("bag_path"));

  bag_data_ = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  parameters_ = std::make_shared<Parameters>();
  parameters_->resample_num = declare_parameter<int>("resample_num");
  parameters_->time_resolution = declare_parameter<double>("time_resolution");
  parameters_->w_lat_comfortability = declare_parameter<double>("weight.lat_comfortability");
  parameters_->w_lon_comfortability = declare_parameter<double>("weight.lon_comfortability");
  parameters_->w_efficiency = declare_parameter<double>("weight.efficiency");
  parameters_->w_safety = declare_parameter<double>("weight.safety");
  parameters_->grid = declare_parameter<std::vector<double>>("grid_seach.grid");
  parameters_->target_state.lat_positions =
    declare_parameter<std::vector<double>>("target_state.lateral_positions");
  parameters_->target_state.lat_velocities =
    declare_parameter<std::vector<double>>("target_state.lateral_velocities");
  parameters_->target_state.lat_accelerations =
    declare_parameter<std::vector<double>>("target_state.lateral_accelerations");
  parameters_->target_state.lon_positions =
    declare_parameter<std::vector<double>>("target_state.longitudinal_positions");
  parameters_->target_state.lon_velocities =
    declare_parameter<std::vector<double>>("target_state.longitudinal_velocities");
  parameters_->target_state.lon_accelerations =
    declare_parameter<std::vector<double>>("target_state.longitudinal_accelerations");
}

void BehaviorAnalyzerNode::update(const std::shared_ptr<BagData> & bag_data) const
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(TOPIC::TF);
  filter.topics.emplace_back(TOPIC::ODOMETRY);
  filter.topics.emplace_back(TOPIC::ACCELERATION);
  filter.topics.emplace_back(TOPIC::OBJECTS);
  filter.topics.emplace_back(TOPIC::STEERING);
  filter.topics.emplace_back(TOPIC::TRAJECTORY);
  reader_.set_filter(filter);

  bag_data->update(0.1 * 1e9);

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
  if (req->data) {
    timer_->reset();
    RCLCPP_INFO(get_logger(), "start evaluation.");
  } else {
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "stop evaluation.");
  }
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

void BehaviorAnalyzerNode::weight(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "start weight grid seach.");

  double minimum_mse = std::numeric_limits<double>::max();
  double best_w0 = 0.0;
  double best_w1 = 0.0;
  double best_w2 = 0.0;
  double best_w3 = 0.0;

  for (const auto & w_lat_comfortability : parameters_->grid) {
    for (const auto & w_lon_comfortability : parameters_->grid) {
      for (const auto & w_efficiency : parameters_->grid) {
        for (const auto & w_safety : parameters_->grid) {
          const auto mse =
            search(w_lat_comfortability, w_lon_comfortability, w_efficiency, w_safety);
          if (mse < minimum_mse) {
            minimum_mse = mse;
            best_w0 = w_lat_comfortability;
            best_w1 = w_lon_comfortability;
            best_w2 = w_efficiency;
            best_w3 = w_safety;
          }
          std::cout << "---result---" << std::endl;
          std::cout << "    MSE:" << mse << " [w0]:" << w_lat_comfortability
                    << " [w1]:" << w_lon_comfortability << " [w2]:" << w_efficiency
                    << " [w3]:" << w_safety << std::endl;
          std::cout << "MIN MSE:" << minimum_mse << " [w0]:" << best_w0 << " [w1]:" << best_w1
                    << " [w2]:" << best_w2 << " [w3]:" << best_w3 << std::endl;
        }
      }
    }
  }
  RCLCPP_INFO(get_logger(), "finish weight grid seach.");

  res->success = true;
}

double BehaviorAnalyzerNode::search(
  const double w0, const double w1, const double w2, const double w3) const
{
  reader_.seek(0);
  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  auto p_tmp = parameters_;
  p_tmp->w_lat_comfortability = w0;
  p_tmp->w_lon_comfortability = w1;
  p_tmp->w_efficiency = w2;
  p_tmp->w_safety = w3;

  double mse = 0.0;
  while (reader_.has_next()) {
    update(bag_data);

    const auto data_set = std::make_shared<DataSet>(bag_data, vehicle_info_, p_tmp);
    const auto mean_square_error = data_set->loss();
    if (mean_square_error.has_value()) {
      mse += mean_square_error.value();
    }
  }

  return mse;
}

void BehaviorAnalyzerNode::analyze(const std::shared_ptr<BagData> & bag_data) const
{
  const auto data_set = std::make_shared<DataSet>(bag_data, vehicle_info_, parameters_);

  const auto opt_tf = std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
                        ->get(bag_data->timestamp);
  if (opt_tf.has_value()) {
    pub_tf_->publish(opt_tf.value());
  }

  const auto opt_objects =
    std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS))
      ->get(bag_data->timestamp);
  if (opt_objects.has_value()) {
    pub_objects_->publish(opt_objects.value());
  }

  const auto opt_trajectory =
    std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
      ->get(bag_data->timestamp);
  if (opt_trajectory.has_value()) {
    pub_trajectory_->publish(opt_trajectory.value());
  }

  metrics(data_set);

  score(data_set);

  visualize(data_set);

  print(data_set);
}

void BehaviorAnalyzerNode::metrics(const std::shared_ptr<DataSet> & data_set) const
{
  {
    Float32MultiArrayStamped msg{};

    msg.stamp = now();
    msg.data.resize(static_cast<size_t>(METRIC::SIZE) * parameters_->resample_num);

    const auto set_metrics = [&msg, this](const auto & data, const auto metric_type) {
      const auto offset = static_cast<size_t>(metric_type) * parameters_->resample_num;
      const auto metric = data.values.at(metric_type);
      std::copy(metric.begin(), metric.end(), msg.data.begin() + offset);
    };

    set_metrics(data_set->manual, METRIC::LATERAL_ACCEL);
    set_metrics(data_set->manual, METRIC::LONGITUDINAL_JERK);
    set_metrics(data_set->manual, METRIC::TRAVEL_DISTANCE);
    set_metrics(data_set->manual, METRIC::MINIMUM_TTC);

    pub_manual_metrics_->publish(msg);
  }

  const auto autoware_trajectory = data_set->sampling.autoware();
  if (autoware_trajectory.has_value()) {
    Float32MultiArrayStamped msg{};

    msg.stamp = now();
    msg.data.resize(static_cast<size_t>(METRIC::SIZE) * parameters_->resample_num);

    const auto set_metrics = [&msg, this](const auto & data, const auto metric_type) {
      const auto offset = static_cast<size_t>(metric_type) * parameters_->resample_num;
      const auto metric = data.values.at(metric_type);
      std::copy(metric.begin(), metric.end(), msg.data.begin() + offset);
    };

    set_metrics(autoware_trajectory.value(), METRIC::LATERAL_ACCEL);
    set_metrics(autoware_trajectory.value(), METRIC::LONGITUDINAL_JERK);
    set_metrics(autoware_trajectory.value(), METRIC::TRAVEL_DISTANCE);
    set_metrics(autoware_trajectory.value(), METRIC::MINIMUM_TTC);

    pub_system_metrics_->publish(msg);
  }
}

void BehaviorAnalyzerNode::score(const std::shared_ptr<DataSet> & data_set) const
{
  {
    Float32MultiArrayStamped msg{};

    msg.stamp = now();
    msg.data.resize(static_cast<size_t>(SCORE::SIZE));

    const auto set_reward = [&msg](const auto & data, const auto score_type) {
      msg.data.at(static_cast<size_t>(score_type)) = static_cast<float>(data.scores.at(score_type));
    };

    set_reward(data_set->manual, SCORE::LONGITUDINAL_COMFORTABILITY);
    set_reward(data_set->manual, SCORE::LATERAL_COMFORTABILITY);
    set_reward(data_set->manual, SCORE::EFFICIENCY);
    set_reward(data_set->manual, SCORE::SAFETY);

    pub_manual_score_->publish(msg);
  }

  const auto autoware_trajectory = data_set->sampling.autoware();
  if (autoware_trajectory.has_value()) {
    Float32MultiArrayStamped msg{};

    msg.stamp = now();
    msg.data.resize(static_cast<size_t>(SCORE::SIZE));

    const auto set_reward = [&msg](const auto & data, const auto score_type) {
      msg.data.at(static_cast<size_t>(score_type)) = static_cast<float>(data.scores.at(score_type));
    };

    set_reward(autoware_trajectory.value(), SCORE::LONGITUDINAL_COMFORTABILITY);
    set_reward(autoware_trajectory.value(), SCORE::LATERAL_COMFORTABILITY);
    set_reward(autoware_trajectory.value(), SCORE::EFFICIENCY);
    set_reward(autoware_trajectory.value(), SCORE::SAFETY);

    pub_system_score_->publish(msg);
  }
}

void BehaviorAnalyzerNode::visualize(const std::shared_ptr<DataSet> & data_set) const
{
  MarkerArray msg;

  size_t i = 0;
  for (const auto & point : data_set->manual.odometry_history) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "manual", i++, Marker::ARROW,
      createMarkerScale(0.7, 0.3, 0.3), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.pose = point.pose.pose;
    msg.markers.push_back(marker);
  }

  for (const auto & trajectory : data_set->sampling.data) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "candidates", i++, Marker::LINE_STRIP,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(0.0, 0.0, 1.0, 0.999));
    if (!trajectory.feasible()) {
      for (const auto & point : trajectory.points) {
        marker.points.push_back(point.pose.position);
        marker.colors.push_back(createMarkerColor(0.1, 0.1, 0.1, 0.5));
      }
    } else {
      for (const auto & point : trajectory.points) {
        marker.points.push_back(point.pose.position);
        marker.colors.push_back(createMarkerColor(0.0, 0.0, 1.0, 0.999));
      }
    }
    msg.markers.push_back(marker);
  }

  const auto best_trajectory = data_set->sampling.best();
  if (best_trajectory.has_value()) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "best", i++, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & point : best_trajectory.value().points) {
      marker.points.push_back(point.pose.position);
    }
    msg.markers.push_back(marker);
  }

  const auto autoware_trajectory = data_set->sampling.autoware();
  if (autoware_trajectory.has_value()) {
    for (const auto & point : autoware_trajectory.value().points) {
      Marker marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "system", i++, Marker::ARROW,
        createMarkerScale(0.7, 0.3, 0.3), createMarkerColor(1.0, 1.0, 0.0, 0.999));
      marker.pose = point.pose;
      msg.markers.push_back(marker);
    }
  }

  pub_marker_->publish(msg);
}

void BehaviorAnalyzerNode::print(const std::shared_ptr<DataSet> & data_set) const
{
  const auto autoware_trajectory = data_set->sampling.autoware();
  if (!autoware_trajectory.has_value()) {
    return;
  }

  const auto best_trajectory = data_set->sampling.best();
  if (!best_trajectory.has_value()) {
    return;
  }

  std::cout << "---result---" << std::endl;
  std::cout << "[HUMAN] SCORE:" << data_set->manual.total() << std::endl;
  std::cout << "[AUTOWARE] SCORE:" << autoware_trajectory.value().total() << std::endl;
  std::cout << "[SAMPLING] BEST SCORE:" << best_trajectory.value().total() << "("
            << best_trajectory.value().tag << ")" << std::endl;
}

void BehaviorAnalyzerNode::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  update(bag_data_);
  analyze(bag_data_);
}
}  // namespace autoware::behavior_analyzer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_analyzer::BehaviorAnalyzerNode)
