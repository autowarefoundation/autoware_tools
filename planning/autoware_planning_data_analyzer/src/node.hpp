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
#include "evaluation.hpp"
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
  void play(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void rewind(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void next_route(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void weight(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  auto get_route() -> LaneletRoute::ConstSharedPtr;

  void update(const std::shared_ptr<BagData> & bag_data, const double dt) const;

  void analyze(const std::shared_ptr<BagData> & bag_data) const;

  void visualize(const std::shared_ptr<BagEvaluator> & bag_evaluator) const;

  void plot(const std::shared_ptr<BagEvaluator> & bag_evaluator) const;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;

  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  rclcpp::Service<Trigger>::SharedPtr srv_play_;

  rclcpp::Service<Trigger>::SharedPtr srv_rewind_;

  rclcpp::Service<Trigger>::SharedPtr srv_route_;

  rclcpp::Service<Trigger>::SharedPtr srv_weight_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  std::shared_ptr<DataAugmentParameters> data_augument_parameters_;

  std::shared_ptr<trajectory_selector::trajectory_evaluator::EvaluatorParameters>
    evaluator_parameters_;

  mutable std::mutex mutex_;

  mutable rosbag2_cpp::Reader reader_;
};
}  // namespace autoware::behavior_analyzer

#endif  // NODE_HPP_
