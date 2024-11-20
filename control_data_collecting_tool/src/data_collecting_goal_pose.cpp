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

#include <rviz_common/display_context.hpp>  // Include for access to DisplayContext

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rviz_plugins
{
// Constructor: Initializes custom QoS profile
DataCollectingGoalPose::DataCollectingGoalPose() : custom_qos_profile_(rclcpp::QoS(5))
{
}

// Callback function for receiving OperationModeState messages
void DataCollectingGoalPose::onOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  // Update control_applying_ flag based on mode and control status
  if (msg->mode == 3 && msg->is_autoware_control_enabled) {
    control_applying_ = true;
  } else {
    control_applying_ = false;
  }
}

// Destructor
DataCollectingGoalPose::~DataCollectingGoalPose() = default;

// Initialization function for the DataCollectingGoalPose tool in RViz
void DataCollectingGoalPose::onInitialize()
{
  // Call the base class initialization
  GoalTool::onInitialize();
  setName("DataCollectingGoalPose");

  // Initialize ROS 2 node and create the PoseStamped publisher with custom QoS
  auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pose_publisher_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/data_collecting_goal_pose", custom_qos_profile_);

  // Subscription to the operation mode state topic with a QoS of 1
  sub_operation_mode_state_ =
    raw_node->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/system/operation_mode/state", rclcpp::QoS{1},
      std::bind(&DataCollectingGoalPose::onOperationModeState, this, std::placeholders::_1));
}

// Override onPoseSet to set pose only when control is not applying
void DataCollectingGoalPose::onPoseSet(double x, double y, double theta)
{
  // If control is not currently being applied, call the base class's onPoseSet function
  if (!control_applying_) {
    GoalTool::onPoseSet(x, y, theta);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
// Export the plugin class so it can be dynamically loaded by RViz
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DataCollectingGoalPose, rviz_common::Tool)
