// Copyright 2024 Proxima Technology Inc, TIER IV Inc.
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

#include "data_collecting_goal_pose.hpp"

#include <rclcpp/logging.hpp>               // Include for ROS logging
#include <rviz_common/display_context.hpp>  // Include for access to DisplayContext

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rviz_plugins
{
// Constructor: Initializes custom QoS profile
DataCollectingGoalPose::DataCollectingGoalPose() : custom_qos_profile_(rclcpp::QoS(5))
{
  RCLCPP_INFO(rclcpp::get_logger("DataCollectingGoalPose"), "DataCollectingGoalPose initialized");
}

// Callback function for receiving OperationModeState messages
void DataCollectingGoalPose::onOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("DataCollectingGoalPose"),
    "Received OperationModeState: mode=%d, is_autoware_control_enabled=%d", msg->mode,
    msg->is_autoware_control_enabled);

  // Update control_applying_ flag based on mode and control status
  if (msg->mode == 3 && msg->is_autoware_control_enabled) {
    control_applying_ = true;
    RCLCPP_INFO(rclcpp::get_logger("DataCollectingGoalPose"), "Control is applying");
  } else {
    control_applying_ = false;
    RCLCPP_INFO(rclcpp::get_logger("DataCollectingGoalPose"), "Control is NOT applying");
  }
}

// Destructor
DataCollectingGoalPose::~DataCollectingGoalPose()
{
  RCLCPP_INFO(rclcpp::get_logger("DataCollectingGoalPose"), "DataCollectingGoalPose destroyed");
}

// Initialization function for the DataCollectingGoalPose tool in RViz
void DataCollectingGoalPose::onInitialize()
{
  RCLCPP_INFO(
    rclcpp::get_logger("DataCollectingGoalPose"), "Initializing DataCollectingGoalPose tool");

  // Call the base class initialization
  GoalTool::onInitialize();
  setName("DataCollectingGoalPose");

  // Initialize ROS 2 node and create the PoseStamped publisher with custom QoS
  auto raw_node = context_->getRosNodeAbstraction().lock();
  if (!raw_node) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DataCollectingGoalPose"), "Failed to lock ROS node abstraction");
    return;
  }

  auto raw_node_ptr = raw_node->get_raw_node();
  pose_publisher_ = raw_node_ptr->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/data_collecting_goal_pose", custom_qos_profile_);
  RCLCPP_INFO(
    rclcpp::get_logger("DataCollectingGoalPose"),
    "Pose publisher initialized on topic: /data_collecting_goal_pose");

  // Subscription to the operation mode state topic with a QoS of 1
  sub_operation_mode_state_ =
    raw_node_ptr->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/system/operation_mode/state", rclcpp::QoS{1},
      std::bind(&DataCollectingGoalPose::onOperationModeState, this, std::placeholders::_1));
  RCLCPP_INFO(
    rclcpp::get_logger("DataCollectingGoalPose"),
    "Subscribed to topic: /system/operation_mode/state");
}

// Override onPoseSet to set pose only when control is not applying
void DataCollectingGoalPose::onPoseSet(double x, double y, double theta)
{
  RCLCPP_INFO(
    rclcpp::get_logger("DataCollectingGoalPose"),
    "onPoseSet called with x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

  if (control_applying_) {
    RCLCPP_WARN(
      rclcpp::get_logger("DataCollectingGoalPose"),
      "Pose setting ignored because control is applying");
    return;
  }

  // Setup Message to publish
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = rclcpp::Clock().now();
  pose_msg.header.frame_id = "map";
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));

  RCLCPP_INFO(
    rclcpp::get_logger("DataCollectingGoalPose"), "Publishing pose: x=%.2f, y=%.2f, theta=%.2f", x,
    y, theta);

  try {
    pose_publisher_->publish(pose_msg);
    RCLCPP_INFO(
      rclcpp::get_logger("DataCollectingGoalPose"),
      "Pose successfully published to /data_collecting_goal_pose");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DataCollectingGoalPose"), "Failed to publish pose: %s", e.what());
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
// Export the plugin class so it can be dynamically loaded by RViz
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DataCollectingGoalPose, rviz_common::Tool)
