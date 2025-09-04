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
 * measure_tool.cpp
 *
 *  Created on: Aug 8, 2012
 *      Author: gossow
 */

#include "evaluate_position_error_using_line_tool.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/line.hpp"

#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/alt_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>

namespace autoware
{

namespace evaluate_position_error_using_line_rviz_plugin
{

EvaluatePositionErrorUsingLineTool::EvaluatePositionErrorUsingLineTool()
: is_line_started_(false), length_(-1), measurement_count_(0)
{
  shortcut_key_ = 'n';

  color_property_ = new rviz_common::properties::ColorProperty(
    "Line color", Qt::darkYellow, "The topic on which to publish points.", getPropertyContainer(),
    SLOT(updateLineColor()), this);

  initCsvFile();
}

EvaluatePositionErrorUsingLineTool::~EvaluatePositionErrorUsingLineTool()
{
  if (csv_file_.is_open()) {
    csv_file_.close();
    std::cout << "CSV file closed: " << csv_filename_ << std::endl;
  }
}

void EvaluatePositionErrorUsingLineTool::onInitialize()
{
  line_ = std::make_shared<rviz_rendering::BillboardLine>(context_->getSceneManager());
  line_lane_ = std::make_shared<rviz_rendering::BillboardLine>(context_->getSceneManager());
  updateLineColor();

  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");

  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_map_ = raw_node->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&EvaluatePositionErrorUsingLineTool::onMap, this, std::placeholders::_1));

  sub_self_pose_ = raw_node->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&EvaluatePositionErrorUsingLineTool::onSelfPose, this, std::placeholders::_1));
}

void EvaluatePositionErrorUsingLineTool::onMap(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg_ptr)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg_ptr, lanelet_map_ptr_);
}

void EvaluatePositionErrorUsingLineTool::onSelfPose(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg_ptr)
{
  self_pose_ = msg_ptr->pose.pose;
}

void EvaluatePositionErrorUsingLineTool::activate()
{
  is_line_started_ = false;
}

void EvaluatePositionErrorUsingLineTool::deactivate()
{
}

int EvaluatePositionErrorUsingLineTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (lanelet_map_ptr_ == nullptr) {
    std::cerr << "null" << std::endl;
    return 0;
  }

  Ogre::Vector3 pos;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (is_line_started_ && success) {
    line_->clear();
    line_->addPoint(start_, color_);
    line_->addPoint(pos, color_);
    length_ = (start_ - pos).length();
  }

  setStatusMessage();

  if (event.leftUp() && success) {
    processLeftButton(pos);
    return Render;
  }
  if (event.rightUp()) {
    processRightButton();
  }

  return 0;
}

void EvaluatePositionErrorUsingLineTool::updateLineColor()
{
  color_ = rviz_common::properties::qtToOgre(color_property_->getColor());
  // line_->setColor(color);
  color_ = Ogre::ColourValue(0, 1, 0, 1);
  color_lane_ = Ogre::ColourValue(0, 0, 1, 1);

  line_->setLineWidth(0.3);

  // line_lane_->setColor(1,0,0,1);
  line_lane_->setLineWidth(0.15);
}

void EvaluatePositionErrorUsingLineTool::setStatusMessage()
{
  std::stringstream ss;
  if (length_ > 0.0) {
    ss << "[Length: " << length_ << "m] ";
  }

  ss << "Click on two points to measure their distance. Right-click to reset.";
  setStatus(QString(ss.str().c_str()));
}

void EvaluatePositionErrorUsingLineTool::processLeftButton(const Ogre::Vector3 & pos)
{
  if (is_line_started_) {
    end_ = pos;
    line_->clear();
    line_->addPoint(start_, color_);
    line_->addPoint(end_, color_);
    is_line_started_ = false;

    // Calculate direction vector and angle of the line segment
    const double line_dx = end_.x - start_.x;
    const double line_dy = end_.y - start_.y;
    const double line_length = std::sqrt(line_dx * line_dx + line_dy * line_dy);
    const double line_angle = std::atan2(line_dy, line_dx);

    // Normalized direction vector
    const double norm_dx = line_dx / line_length;
    const double norm_dy = line_dy / line_length;

    // Extract Lanelets within 1.5m of the line segment
    lanelet::ConstLanelets nearby_lanes;
    const double search_radius = 1.5;  // 1.5m

    for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
      const std::string type = lanelet.attributeOr(lanelet::AttributeName::Type, "none");
      const std::string subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, "none");
      if (type == "lanelet" && subtype == "road") {
        // Check if boundary points of the Lanelet are near the line segment
        bool is_nearby = false;

        // Check left and right boundaries
        for (const auto & bound : {lanelet.leftBound(), lanelet.rightBound()}) {
          for (const auto & point : bound) {
            // Calculate distance from point to line segment
            const double px = point.x() - start_.x;
            const double py = point.y() - start_.y;

            // Projection length onto the line segment
            const double projection = px * norm_dx + py * norm_dy;

            if (projection >= 0 && projection <= line_length) {  // Within the line segment
              // Perpendicular distance from the line segment
              const double distance = std::abs(px * norm_dy - py * norm_dx);
              if (distance <= search_radius) {
                is_nearby = true;
                break;
              }
            } else {
              // Outside line segment, check distance to nearest endpoint
              double distance_to_start = std::sqrt(px * px + py * py);
              const double end_px = point.x() - end_.x;
              const double end_py = point.y() - end_.y;
              double distance_to_end = std::sqrt(end_px * end_px + end_py * end_py);
              double min_distance = std::min(distance_to_start, distance_to_end);
              if (min_distance <= search_radius) {
                is_nearby = true;
                break;
              }
            }
          }
          if (is_nearby) break;
        }

        if (is_nearby) {
          nearby_lanes.push_back(lanelet);
        }
      }
    }

    // Extract boundaries with similar slopes to the line segment and calculate approximation line
    std::vector<lanelet::BasicPoint2d> candidate_points;

    for (const auto & lanelet : nearby_lanes) {
      // Process left and right boundaries
      for (const auto & bound : {lanelet.leftBound(), lanelet.rightBound()}) {
        // Check if all segments in the boundary have similar slopes
        // Only check segments that are within search_radius of the line segment
        bool all_segments_valid = true;

        for (size_t i = 1; i < bound.size() && all_segments_valid; ++i) {
          const double seg_dx = bound[i].x() - bound[i - 1].x();
          const double seg_dy = bound[i].y() - bound[i - 1].y();
          const double seg_length = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

          if (seg_length > 1e-6) {  // Valid segment
            // Check if this segment is within search_radius of the line segment
            bool segment_in_range = false;

            // Check both endpoints of the segment
            for (const auto & point : {bound[i - 1], bound[i]}) {
              const double px = point.x() - start_.x;
              const double py = point.y() - start_.y;

              // Projection length onto the line segment
              const double projection = px * norm_dx + py * norm_dy;

              if (projection >= 0 && projection <= line_length) {  // Within the line segment
                // Perpendicular distance from the line segment
                const double distance = std::abs(px * norm_dy - py * norm_dx);
                if (distance <= search_radius) {
                  segment_in_range = true;
                  break;
                }
              } else {
                // Outside line segment, check distance to nearest endpoint
                double distance_to_start = std::sqrt(px * px + py * py);
                const double end_px = point.x() - end_.x;
                const double end_py = point.y() - end_.y;
                double distance_to_end = std::sqrt(end_px * end_px + end_py * end_py);
                double min_distance = std::min(distance_to_start, distance_to_end);
                if (min_distance <= search_radius) {
                  segment_in_range = true;
                  break;
                }
              }
            }

            // Only check angle if the segment is within range
            if (segment_in_range) {
              const double seg_angle = std::atan2(seg_dy, seg_dx);

              // Calculate angle difference for this segment
              double angle_diff = std::remainder(seg_angle - line_angle, 2 * M_PI);
              angle_diff = std::abs(angle_diff);
              if (angle_diff > M_PI / 2) {
                angle_diff = M_PI - angle_diff;  // Consider reverse direction
              }

              // If any segment has large angle difference, reject this boundary
              if (angle_diff > M_PI / 36) {  // 5 degrees
                all_segments_valid = false;
              }
            }
          }
        }

        // If all segments are valid, extract points from this boundary
        if (all_segments_valid) {
          // Extract points within the line segment region
          for (const auto & point : bound) {
            const double px = point.x() - start_.x;
            const double py = point.y() - start_.y;

            // Projection length onto the line segment
            const double projection = px * norm_dx + py * norm_dy;

            if (projection >= 0 && projection <= line_length) {  // Within the line segment
              // Perpendicular distance from the line segment
              const double distance = std::abs(px * norm_dy - py * norm_dx);
              if (distance <= search_radius) {
                candidate_points.push_back(lanelet::BasicPoint2d(point.x(), point.y()));
              }
            } else {
              // Outside line segment, check distance to nearest endpoint
              double distance_to_start = std::sqrt(px * px + py * py);
              const double end_px = point.x() - end_.x;
              const double end_py = point.y() - end_.y;
              double distance_to_end = std::sqrt(end_px * end_px + end_py * end_py);
              double min_distance = std::min(distance_to_start, distance_to_end);
              if (min_distance <= search_radius) {
                candidate_points.push_back(lanelet::BasicPoint2d(point.x(), point.y()));
              }
            }
          }
        }
      }
    }

    // Calculate approximation line from extracted points using least squares method
    Ogre::Vector3 lane_seg_start, lane_seg_end;
    bool is_no_lane_points = true;

    if (candidate_points.size() >= 2) {
      is_no_lane_points = false;
      // Calculate approximation line using least squares method
      double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
      const size_t n = candidate_points.size();

      for (const auto & point : candidate_points) {
        sum_x += point.x();
        sum_y += point.y();
        sum_xx += point.x() * point.x();
        sum_xy += point.x() * point.y();
      }

      // Calculate line parameters: y = ax + b
      const double denominator = n * sum_xx - sum_x * sum_x;

      if (std::abs(denominator) > 1e-10) {
        // Normal case: y = ax + b
        const double line_a = (n * sum_xy - sum_x * sum_y) / denominator;
        const double line_b = (sum_y - line_a * sum_x) / n;

        // Calculate perpendicular feet from start and end points to the approximation line
        const double denom = 1.0 + line_a * line_a;

        // Perpendicular foot from start point
        const double start_foot_x = (start_.x + line_a * (start_.y - line_b)) / denom;
        const double start_foot_y = line_a * start_foot_x + line_b;

        // Perpendicular foot from end point
        const double end_foot_x = (end_.x + line_a * (end_.y - line_b)) / denom;
        const double end_foot_y = line_a * end_foot_x + line_b;

        lane_seg_start = Ogre::Vector3(start_foot_x, start_foot_y, 100);
        lane_seg_end = Ogre::Vector3(end_foot_x, end_foot_y, 100);
      } else {
        // Nearly vertical line, use x = cy + d instead
        double sum_yy = 0.0, sum_yx = 0.0;
        for (const auto & point : candidate_points) {
          sum_yy += point.y() * point.y();
          sum_yx += point.y() * point.x();
        }
        const double denom_y = n * sum_yy - sum_y * sum_y;
        if (std::abs(denom_y) > 1e-10) {
          const double line_c = (n * sum_yx - sum_y * sum_x) / denom_y;
          const double line_d = (sum_x - line_c * sum_y) / n;

          // Calculate perpendicular feet for vertical-ish line
          const double start_foot_y = start_.y;
          const double start_foot_x = line_c * start_foot_y + line_d;
          const double end_foot_y = end_.y;
          const double end_foot_x = line_c * end_foot_y + line_d;

          lane_seg_start = Ogre::Vector3(start_foot_x, start_foot_y, 100);
          lane_seg_end = Ogre::Vector3(end_foot_x, end_foot_y, 100);
        } else {
          // Degenerate case: all points are the same
          lane_seg_start = Ogre::Vector3(candidate_points[0].x(), candidate_points[0].y(), 100);
          lane_seg_end = Ogre::Vector3(candidate_points[0].x(), candidate_points[0].y(), 100);
        }
      }
    }

    // Stop line processing
    lanelet::BasicPoint2d point2d((start_.x + end_.x) / 2.0, (start_.y + end_.y) / 2.0);

    auto search_func =
      [](const lanelet::BoundingBox2d & /*box*/, const lanelet::LineString3d & linestring) {
        return linestring.attributeOr(lanelet::AttributeName::Type, "") == std::string("stop_line");
      };

    lanelet::Optional<lanelet::LineString3d> stop_line =
      lanelet_map_ptr_->lineStringLayer.nearestUntil(point2d, search_func);

    Ogre::Vector3 stop_line_seg_start, stop_line_seg_end;
    bool stop_line_found = false;

    if (stop_line) {
      stop_line_seg_start =
        Ogre::Vector3(stop_line.get().front().x(), stop_line.get().front().y(), 100);
      stop_line_seg_end =
        Ogre::Vector3(stop_line.get().back().x(), stop_line.get().back().y(), 100);
      stop_line_found = true;
    }

    enum class LINE_TYPE { LANE, STOP_LINE, ERROR };

    LINE_TYPE line_type;
    if (!is_no_lane_points) {
      seg_start_ = lane_seg_start;
      seg_end_ = lane_seg_end;
      line_type = LINE_TYPE::LANE;
    } else if (stop_line_found) {
      seg_start_ = stop_line_seg_start;
      seg_end_ = stop_line_seg_end;
      line_type = LINE_TYPE::STOP_LINE;
    } else {
      line_type = LINE_TYPE::ERROR;
    }

    line_lane_->clear();
    line_lane_->addPoint(seg_start_, color_lane_);
    line_lane_->addPoint(seg_end_, color_lane_);

    const double yaw_line = std::atan2(end_.y - start_.y, end_.x - start_.x);
    const double yaw_lane = std::atan2(seg_end_.y - seg_start_.y, seg_end_.x - seg_start_.x);
    double yaw = std::remainder(yaw_line - yaw_lane, 2 * M_PI);
    if (yaw > M_PI / 2) {
      yaw = M_PI - yaw;
    } else if (yaw < -M_PI / 2) {
      yaw = -M_PI - yaw;
    }

    const double distance_selfpose_to_line = autoware_utils::distance(
      autoware_utils::alt::Point2d(self_pose_.position.x, self_pose_.position.y),
      autoware_utils::alt::Point2d(start_.x, start_.y),
      autoware_utils::alt::Point2d(end_.x, end_.y));

    const double distance_selfpose_to_lane = autoware_utils::distance(
      autoware_utils::alt::Point2d(self_pose_.position.x, self_pose_.position.y),
      autoware_utils::alt::Point2d(seg_start_.x, seg_start_.y),
      autoware_utils::alt::Point2d(seg_end_.x, seg_end_.y));

    // Error calculation
    double x_error = 0.0;
    double y_error = 0.0;

    if (line_type == LINE_TYPE::STOP_LINE) {
      // For stop_line case, calculate distance as x_error
      x_error = distance_selfpose_to_line - distance_selfpose_to_lane;
    } else if (line_type == LINE_TYPE::LANE) {
      // For lane case, calculate only y_error
      y_error = distance_selfpose_to_line - distance_selfpose_to_lane;
    } else if (line_type == LINE_TYPE::ERROR) {
      // None
    }

    // Display number of extracted points
    std::cout << "Extracted " << candidate_points.size() << " candidate points from nearby lanelets"
              << std::endl;
    std::cout << "Map line approximation: start(" << seg_start_.x << ", " << seg_start_.y
              << ") -> end(" << seg_end_.x << ", " << seg_end_.y << ")" << std::endl;

    if (line_type == LINE_TYPE::STOP_LINE) {
      std::cout << "x_error: " << x_error << "[m], y_error: " << "none"
                << "[m], yaw_error: " << yaw / M_PI * 180.0 << "[deg]" << std::endl;
      writeToCsv(x_error, std::numeric_limits<double>::quiet_NaN(), yaw / M_PI * 180.0);
    } else if (line_type == LINE_TYPE::LANE) {
      std::cout << "x_error: " << "none" << "[m], y_error: " << y_error
                << "[m], yaw_error: " << yaw / M_PI * 180.0 << "[deg]" << std::endl;
      writeToCsv(std::numeric_limits<double>::quiet_NaN(), y_error, yaw / M_PI * 180.0);
    } else if (line_type == LINE_TYPE::ERROR) {
      std::cout << "x_error: " << "none" << "[m], y_error: " << "none"
                << "[m], yaw_error: " << "none" << "[deg]" << std::endl;
      // Do not write to CSV for error cases
    }

  } else {
    start_ = pos;
    is_line_started_ = true;
  }
}

void EvaluatePositionErrorUsingLineTool::processRightButton()
{
  is_line_started_ = false;
  // line_->setVisible(false);
  line_->clear();
  line_lane_->clear();
}

void EvaluatePositionErrorUsingLineTool::initCsvFile()
{
  // Create output directory if it doesn't exist
  const char* home_dir = std::getenv("HOME");
  std::string output_dir;
  if (home_dir) {
    output_dir = std::string(home_dir) + "/evaluate_position_error_using_line_tool";
  } else {
    output_dir = "./evaluate_position_error_using_line_tool";
  }
  
  // Create directory (recursive creation)
  if (mkdir(output_dir.c_str(), 0755) != 0 && errno != EEXIST) {
    std::cerr << "Failed to create directory: " << output_dir << std::endl;
  }

  // Generate filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);

  // Create timestamp string for filenames and directories
  std::stringstream timestamp_ss;
  timestamp_ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  capture_timestamp_ = timestamp_ss.str();
  // Generate CSV filename with automatic numbering to avoid conflicts
  std::string base_filename = output_dir + "/position_error_evaluation_" + capture_timestamp_;
  csv_filename_ = base_filename + ".csv";
  
  // If file exists, add sequential number
  int file_counter = 1;
  while (std::ifstream(csv_filename_).good()) {
    std::stringstream numbered_ss;
    numbered_ss << base_filename << "_" << std::setfill('0') << std::setw(2) << file_counter << ".csv";
    csv_filename_ = numbered_ss.str();
    file_counter++;
  }

  // Open CSV file and write header
  csv_file_.open(csv_filename_);
  if (csv_file_.is_open()) {
    csv_file_ << "measurement_number,x_error,y_error,yaw_error" << std::endl;
    std::cout << "CSV output file created: " << csv_filename_ << std::endl;
  } else {
    std::cerr << "Failed to create CSV file: " << csv_filename_ << std::endl;
  }
}

void EvaluatePositionErrorUsingLineTool::writeToCsv(
  double x_error, double y_error, double yaw_error)
{
  if (csv_file_.is_open()) {
    measurement_count_++;
    csv_file_ << measurement_count_ << "," << std::fixed << std::setprecision(6) << x_error << ","
              << y_error << "," << yaw_error << std::endl;
    csv_file_.flush();  // Ensure data is written immediately
    std::cout << "Measurement " << measurement_count_ << " written to CSV" << std::endl;
  }
}

}  // namespace evaluate_position_error_using_line_rviz_plugin
}  // namespace autoware

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::evaluate_position_error_using_line_rviz_plugin::EvaluatePositionErrorUsingLineTool,
  rviz_common::Tool)
