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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "bag_handler.hpp"
#include "data_structs.hpp"
#include "evaluation.hpp"
#include "matplotlibcpp.h"
#include "rosbag2_cpp/reader.hpp"
#include "type_alias.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_analyzer
{
class BehaviorAnalyzerNode : public rclcpp::Node
{
public:
  explicit BehaviorAnalyzerNode(const rclcpp::NodeOptions & node_options);

private:
  void on_timer();

  void play(const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res);

  void rewind(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void next_route(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void weight(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  auto get_route() -> LaneletRoute::ConstSharedPtr;

  void update(const std::shared_ptr<BagData> & bag_data, const double dt) const;

  void analyze(const std::shared_ptr<BagData> & bag_data) const;

  void metrics(const std::shared_ptr<Evaluator> & data_set) const;

  void score(const std::shared_ptr<Evaluator> & data_set) const;

  void visualize(const std::shared_ptr<Evaluator> & data_set) const;

  void plot(const std::shared_ptr<Evaluator> & data_set) const;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::TimerBase::SharedPtr timer_plot_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_manual_metrics_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_system_metrics_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_manual_score_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_system_score_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Service<SetBool>::SharedPtr srv_play_;
  rclcpp::Service<Trigger>::SharedPtr srv_rewind_;
  rclcpp::Service<Trigger>::SharedPtr srv_route_;
  rclcpp::Service<Trigger>::SharedPtr srv_weight_;

  vehicle_info_utils::VehicleInfo vehicle_info_;

  std::shared_ptr<BagData> bag_data_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<Parameters> parameters_;

  mutable std::shared_ptr<TrajectoryPoints> previous_;

  mutable std::mutex mutex_;

  mutable std::vector<std::vector<double>> buffer_;

  mutable rosbag2_cpp::Reader reader_;

  mutable size_t count_;
};
}  // namespace autoware::behavior_analyzer

#endif  // NODE_HPP_
