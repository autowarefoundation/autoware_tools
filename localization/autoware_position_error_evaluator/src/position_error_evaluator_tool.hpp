// Copyright 2018-2025 Autoware Foundation
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

#ifndef POSITION_ERROR_EVALUATOR_TOOL_HPP_
#define POSITION_ERROR_EVALUATOR_TOOL_HPP_

#include "rviz_common/tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <QCursor>  // NOLINT cpplint cannot handle include order
#include <QKeyEvent>
#include <rclcpp/rclcpp.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <OgreVector.h>
#include <lanelet2_core/LaneletMap.h>
#include <sys/stat.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

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
namespace position_error_evaluator
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC PositionErrorEvaluatorTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PositionErrorEvaluatorTool();
  ~PositionErrorEvaluatorTool();

  void onInitialize() override;

  void onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg_ptr);
  void onSelfPose(const nav_msgs::msg::Odometry::ConstSharedPtr & msg_ptr);

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

public Q_SLOTS:
  void updateLineColor();

private:
  void setStatusMessage();
  void processLeftButton(const Ogre::Vector3 & pos);
  void processRightButton();
  void undoLastMeasurement();
  void initCsvFile();
  void writeToCsv(double x_error, double y_error, double yaw_error);
  void initTrajectoryFile();
  void writeTrajectoryToCsv(double x, double y, double z, double yaw);
  void initScreenshotDirectory();
  void takeScreenshotAfterMeasurement();

  // Constants
  static constexpr double LINE_WIDTH_MAIN = 0.3;
  static constexpr double LINE_WIDTH_LANE = 0.15;
  static constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;
  static constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;
  static constexpr double ANGLE_THRESHOLD_DEGREES = 5.0;
  static constexpr double SEARCH_RADIUS_METERS = 1.5;
  static constexpr int SCREENSHOT_DELAY_MICROSECONDS = 200000;  // 200ms
  static constexpr mode_t DIRECTORY_PERMISSIONS = 0755;
  static constexpr int FILENAME_SEQUENCE_WIDTH = 2;

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

  // Trajectory CSV output related
  std::ofstream trajectory_file_;
  std::string trajectory_filename_;

  // Screenshot related
  std::string screenshot_directory_;
  std::string capture_timestamp_;

  // Undo functionality
  bool has_last_measurement_;
  std::string last_screenshot_path_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
};

}  // namespace position_error_evaluator
}  // namespace autoware

#endif  // POSITION_ERROR_EVALUATOR_TOOL_HPP_
