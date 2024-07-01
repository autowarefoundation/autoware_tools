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

#ifndef DATA_COLLECTING_AREA_SELECTION_HPP_
#define DATA_COLLECTING_AREA_SELECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_default_plugins/tools/move/move_tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <memory>

namespace rviz_rendering
{
class Shape;
}

namespace rviz_default_plugins
{
namespace tools
{
class MoveTool;
}
}  // namespace rviz_default_plugins
namespace rviz_plugins
{

class DataCollectingAreaSelectionTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  DataCollectingAreaSelectionTool();
  ~DataCollectingAreaSelectionTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event);
  virtual void update(float wall_dt, float ros_dt);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_operation_mode_state_;

  void onOperationModeState(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);

  Ogre::Vector3 start_pos;

  rviz_default_plugins::tools::MoveTool * move_tool_;
  bool selecting_;
  bool control_applying_;
  int sel_start_x_;
  int sel_start_y_;
  int sel_end_x_;
  int sel_end_y_;

  rviz_common::interaction::M_Picked highlight_;

  bool moving_;
};

}  // namespace rviz_plugins

#endif  // DATA_COLLECTING_AREA_SELECTION_HPP_
