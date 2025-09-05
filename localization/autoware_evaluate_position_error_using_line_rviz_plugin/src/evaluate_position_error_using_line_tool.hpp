// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * measure_tool.h
 *
 *  Created on: Aug 8, 2012
 *      Author: gossow
 */

#ifndef EVALUATE_POSITION_ERROR_USING_LINE_TOOL_HPP_
#define EVALUATE_POSITION_ERROR_USING_LINE_TOOL_HPP_

#include "rviz_common/tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <QCursor>  // NOLINT cpplint cannot handle include order
#include <rclcpp/rclcpp.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <OgreVector.h>
#include <lanelet2_core/LaneletMap.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

namespace rviz_rendering
{
class Line;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
}
}  // namespace rviz_common

namespace autoware
{
namespace evaluate_position_error_using_line_rviz_plugin
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC EvaluatePositionErrorUsingLineTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  EvaluatePositionErrorUsingLineTool();
  ~EvaluatePositionErrorUsingLineTool();

  void onInitialize() override;

  void onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg_ptr);
  void onSelfPose(const nav_msgs::msg::Odometry::ConstSharedPtr & msg_ptr);

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:
  void updateLineColor();

private:
  void setStatusMessage();
  void processLeftButton(const Ogre::Vector3 & pos);
  void processRightButton();
  void initCsvFile();
  void writeToCsv(double x_error, double y_error, double yaw_error);
  void initScreenshotDirectory();
  void takeScreenshotAfterMeasurement();

  rviz_common::properties::ColorProperty * color_property_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_self_pose_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::shared_ptr<rviz_rendering::BillboardLine> line_;
  std::shared_ptr<rviz_rendering::BillboardLine> line_lane_;

  Ogre::ColourValue color_;
  Ogre::ColourValue color_lane_;

  Ogre::Vector3 start_;
  Ogre::Vector3 end_;
  Ogre::Vector3 seg_start_;
  Ogre::Vector3 seg_end_;

  geometry_msgs::msg::Pose self_pose_;
  bool is_line_started_;
  float length_;

  // CSV output related
  std::ofstream csv_file_;
  std::string csv_filename_;
  int measurement_count_;

  // Screenshot related
  std::string screenshot_directory_;
  std::string capture_timestamp_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
};

}  // namespace evaluate_position_error_using_line_rviz_plugin
}  // namespace autoware

#endif  // EVALUATE_POSITION_ERROR_USING_LINE_TOOL_HPP_
