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

#ifndef DRIVING_ENVIRONMENT_ANALYZER__NODE_HPP_
#define DRIVING_ENVIRONMENT_ANALYZER__NODE_HPP_

#include "rosbag2_cpp/reader.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace driving_environment_analyzer
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class DrivingEnvironmentAnalyzer : public rclcpp::Node
{
public:
  explicit DrivingEnvironmentAnalyzer(const rclcpp::NodeOptions & node_options);

private:
  bool isDataReady(const bool use_map_in_bag);
  void onMap(const LaneletMapBin::ConstSharedPtr map_msg);
  void analyze();

  bool has_map_data_{false};

  std::vector<LaneletRoute> route_msgs_;

  route_handler::RouteHandler route_handler_;
  LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::TimerBase::SharedPtr timer_;
  rosbag2_cpp::Reader reader_;
};
}  // namespace driving_environment_analyzer

#endif  // DRIVING_ENVIRONMENT_ANALYZER__NODE_HPP_
