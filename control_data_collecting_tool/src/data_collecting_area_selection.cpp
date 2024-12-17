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

#include "data_collecting_area_selection.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_default_plugins/tools/move/move_tool.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

#include <memory>

using std::placeholders::_1;

namespace rviz_plugins
{
DataCollectingAreaSelectionTool::DataCollectingAreaSelectionTool() : rviz_common::Tool()
{
  move_tool_ = new rviz_default_plugins::tools::MoveTool();
  selecting_ = false;
  control_applying_ = false;
  sel_start_x_ = 0;
  sel_start_y_ = 0;
  sel_end_x_ = 0;
  sel_end_y_ = 0;
  moving_ = false;
}

DataCollectingAreaSelectionTool::~DataCollectingAreaSelectionTool()
{
  delete move_tool_;
}

void DataCollectingAreaSelectionTool::onOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  if (msg->mode == 3 && msg->is_autoware_control_enabled) {
    control_applying_ = true;
  } else {
    control_applying_ = false;
  }
}

void DataCollectingAreaSelectionTool::onInitialize()
{
  nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  polygon_pub_ = nh_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/data_collecting_area", rclcpp::QoS(10));

  sub_operation_mode_state_ =
    nh_->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/system/operation_mode/state", rclcpp::QoS{1},
      std::bind(&DataCollectingAreaSelectionTool::onOperationModeState, this, _1));

  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

  move_tool_->initialize(context_);
}

void DataCollectingAreaSelectionTool::activate()
{
  setStatus("Click and drag to select data collecting area on the screen.");
  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;
  moving_ = false;
}

void DataCollectingAreaSelectionTool::deactivate()
{
  context_->getSelectionManager()->removeHighlight();
}

void DataCollectingAreaSelectionTool::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;
  auto sel_manager = context_->getSelectionManager();

  if (!selecting_) {
    sel_manager->removeHighlight();
  }
}

int DataCollectingAreaSelectionTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  auto sel_manager = context_->getSelectionManager();
  auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  int flags = 0;

  if (event.alt()) {
    moving_ = true;
    selecting_ = false;
  } else {
    moving_ = false;

    if (event.leftDown()) {
      selecting_ = true;
      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
      start_pos[0] = point_projection_on_xy_plane.second[0];
      start_pos[1] = point_projection_on_xy_plane.second[1];
      start_pos[2] = point_projection_on_xy_plane.second[2];
    }
  }

  if (selecting_) {
    sel_manager->highlight(
      event.panel->getRenderWindow(), sel_start_x_, sel_start_y_, event.x, event.y);

    if (event.leftUp()) {
      sel_end_x_ = event.x;
      sel_end_y_ = event.y;
      auto tmp_point_projection_on_xy_plane1 =
        projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), sel_start_x_, sel_end_y_);
      auto tmp_point_projection_on_xy_plane2 =
        projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), sel_end_x_, sel_start_y_);

      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header.stamp = nh_->now();
      polygon_msg.header.frame_id = "map";
      polygon_msg.polygon.points.push_back(generatePoint(start_pos[0], start_pos[1], start_pos[2]));
      polygon_msg.polygon.points.push_back(generatePoint(
        tmp_point_projection_on_xy_plane1.second[0], tmp_point_projection_on_xy_plane1.second[1],
        tmp_point_projection_on_xy_plane1.second[2]));
      polygon_msg.polygon.points.push_back(generatePoint(
        point_projection_on_xy_plane.second[0], point_projection_on_xy_plane.second[1],
        point_projection_on_xy_plane.second[2]));
      polygon_msg.polygon.points.push_back(generatePoint(
        tmp_point_projection_on_xy_plane2.second[0], tmp_point_projection_on_xy_plane2.second[1],
        tmp_point_projection_on_xy_plane2.second[2]));
      if (!control_applying_) {
        polygon_pub_->publish(polygon_msg);
      }
      selecting_ = false;
    }

    flags |= Render;
  } else if (moving_) {
    sel_manager->removeHighlight();

    flags = move_tool_->processMouseEvent(event);

    if (event.type == QEvent::MouseButtonRelease) {
      moving_ = false;
    }
  } else {
    sel_manager->highlight(event.panel->getRenderWindow(), event.x, event.y, event.x, event.y);
  }
  return flags;
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DataCollectingAreaSelectionTool, rviz_common::Tool)
