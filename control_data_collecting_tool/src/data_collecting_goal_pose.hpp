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

#ifndef DATA_COLLECTING_GOAL_POSE_HPP_
#define DATA_COLLECTING_GOAL_POSE_HPP_

#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"

#include <rviz_common/tool.hpp>
#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rviz_plugins
{
// DataCollectingGoalPose class that extends GoalTool to add data collection features
class DataCollectingGoalPose : public rviz_default_plugins::tools::GoalTool
{
  Q_OBJECT
public:
  // Constructor
  DataCollectingGoalPose();
  // Destructor
  ~DataCollectingGoalPose();

  // Called during tool initialization in RViz
  void onInitialize() override;

  // Sets the pose using specified coordinates and orientation
  void onPoseSet(double x, double y, double theta) override;

private:
  // Flag to indicate if control is being applied
  bool control_applying_ = false;

  // Subscription to listen for operation mode state messages
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_operation_mode_state_;

  // Callback function to handle received operation mode state messages
  void onOperationModeState(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);

  // Custom QoS profile for message publishing
  rclcpp::QoS custom_qos_profile_;
  // Shared pointer to the raw ROS 2 node used for publishing
  rclcpp::Node::SharedPtr raw_node_;
  // Publisher to send PoseStamped messages to a specific topic
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};
}  // namespace rviz_plugins

#endif  // DATA_COLLECTING_GOAL_POSE_HPP_
