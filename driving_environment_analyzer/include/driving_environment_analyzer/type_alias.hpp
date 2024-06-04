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

#ifndef DRIVING_ENVIRONMENT_ANALYZER__TYPE_ALIAS_HPP_
#define DRIVING_ENVIRONMENT_ANALYZER__TYPE_ALIAS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace driving_environment_analyzer
{

// std
using std::chrono::duration_cast;
using std::chrono::seconds;

// ros2
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using tf2_msgs::msg::TFMessage;

// autoware
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;

}  // namespace driving_environment_analyzer

#endif  // DRIVING_ENVIRONMENT_ANALYZER__TYPE_ALIAS_HPP_
