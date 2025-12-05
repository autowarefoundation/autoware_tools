// Copyright 2025 TIER IV, Inc.
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

#ifndef DATA_TYPES_HPP_
#define DATA_TYPES_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <memory>
#include <string>

namespace autoware::planning_data_analyzer
{

// Type aliases for ROS messages
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using TFMessage = tf2_msgs::msg::TFMessage;
using Odometry = nav_msgs::msg::Odometry;
using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
using AccelWithCovarianceStamped = geometry_msgs::msg::AccelWithCovarianceStamped;
using SteeringReport = autoware_vehicle_msgs::msg::SteeringReport;

// Synchronized data from multiple topics at a specific timestamp
struct SynchronizedData
{
  std::shared_ptr<Odometry> kinematic_state;
  std::shared_ptr<Trajectory> trajectory;
  std::shared_ptr<AccelWithCovarianceStamped> acceleration;
  std::shared_ptr<SteeringReport> steering_status;
  std::shared_ptr<PredictedObjects> objects;
  rclcpp::Time timestamp;
  rclcpp::Time bag_timestamp;
};

// Topic names configuration
struct TopicNames
{
  std::string route_topic;
  std::string odometry_topic;
  std::string trajectory_topic;
  std::string objects_topic;
  std::string tf_topic;
  std::string acceleration_topic;
  std::string steering_topic;
};

}  // namespace autoware::planning_data_analyzer

#endif  // DATA_TYPES_HPP_
