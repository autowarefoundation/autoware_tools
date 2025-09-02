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

namespace autoware
{

namespace evaluate_position_error_using_line_rviz_plugin
{

EvaluatePositionErrorUsingLineTool::EvaluatePositionErrorUsingLineTool()
: is_line_started_(false), length_(-1)
{
  shortcut_key_ = 'n';

  color_property_ = new rviz_common::properties::ColorProperty(
    "Line color", Qt::darkYellow,
    "The topic on which to publish points.",
    getPropertyContainer(), SLOT(updateLineColor()), this);
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


void EvaluatePositionErrorUsingLineTool::onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg_ptr)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg_ptr, lanelet_map_ptr_);
}

void EvaluatePositionErrorUsingLineTool::onSelfPose(const nav_msgs::msg::Odometry::ConstSharedPtr & msg_ptr)
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
  if(lanelet_map_ptr_ == nullptr) {
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
  color_ = Ogre::ColourValue(0,1,0,1);
  color_lane_ = Ogre::ColourValue(0,0,1,1);

  line_->setLineWidth(0.3);

  // line_lane_->setColor(1,0,0,1);
  line_lane_->setLineWidth(0.3);
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



    lanelet::ConstLanelets lanes;
    for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
      const std::string type = lanelet.attributeOr(lanelet::AttributeName::Type, "none");
      const std::string subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, "none");
      if (type == "lanelet" && subtype == "road") {
        lanes.push_back(lanelet);
      }
    }

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = start_.x;
    start_pose.position.y = start_.y;

    lanelet::Lanelet closest_lane_start;
    lanelet::utils::query::getClosestLanelet(lanes, start_pose, &closest_lane_start);

    geometry_msgs::msg::Pose end_pose;
    end_pose.position.x = end_.x;
    end_pose.position.y = end_.y;

    lanelet::Lanelet closest_lane_end;
    lanelet::utils::query::getClosestLanelet(lanes, end_pose, &closest_lane_end);

    const auto lane_right_segment1 = lanelet::utils::getClosestSegment(lanelet::BasicPoint2d(start_.x, start_.y), closest_lane_start.rightBound());
    const auto lane_right_segment2 = lanelet::utils::getClosestSegment(lanelet::BasicPoint2d(end_.x, end_.y), closest_lane_end.rightBound());

    const auto lane_left_segment1 = lanelet::utils::getClosestSegment(lanelet::BasicPoint2d(start_.x, start_.y), closest_lane_start.leftBound());
    const auto lane_left_segment2 = lanelet::utils::getClosestSegment(lanelet::BasicPoint2d(end_.x, end_.y), closest_lane_end.leftBound());

    const double lane_right_distance = ((lane_right_segment1.front().x() + lane_right_segment2.back().x())/2.0 - (start_.x+end_.x)/2.0)
                                      *((lane_right_segment1.front().x() + lane_right_segment2.back().x())/2.0 - (start_.x+end_.x)/2.0)
                                      +((lane_right_segment1.front().y() + lane_right_segment2.back().y())/2.0 - (start_.y+end_.y)/2.0)
                                      *((lane_right_segment1.front().y() + lane_right_segment2.back().y())/2.0 - (start_.y+end_.y)/2.0);
    const double lane_left_distance =  ((lane_left_segment1.front().x() + lane_left_segment2.back().x())/2.0 - (start_.x+end_.x)/2.0)
                                      *((lane_left_segment1.front().x() + lane_left_segment2.back().x())/2.0 - (start_.x+end_.x)/2.0)
                                      +((lane_left_segment1.front().y() + lane_left_segment2.back().y())/2.0 - (start_.y+end_.y)/2.0)
                                      *((lane_left_segment1.front().y() + lane_left_segment2.back().y())/2.0 - (start_.y+end_.y)/2.0);

    const auto lane_segment1 = lane_right_distance < lane_left_distance ? lane_right_segment1 : lane_left_segment1; 
    const auto lane_segment2 = lane_right_distance < lane_left_distance ? lane_right_segment2 : lane_left_segment2;

    const auto lane_seg_start = Ogre::Vector3(lane_segment1.front().x(), lane_segment1.front().y(), 100);
    const auto lane_seg_end = Ogre::Vector3(lane_segment2.back().x(), lane_segment2.back().y(), 100);


    // const auto lanelets =  lanelet_map_ptr_->laneletLayer.nearest(lanelet::BasicPoint2d((start_.x+end_.x)/2.0, (start_.y+end_.y)/2.0) ,1);
    // lanelets.front().rightBound().front().x();
    // lanelets.front().rightBound().front().x();

    lanelet::BasicPoint2d point2d((start_.x+end_.x)/2.0, (start_.y+end_.y)/2.0);

    auto search_func = [](const lanelet::BoundingBox2d& /*box*/, const lanelet::LineString3d& linestring) {
      return linestring.attributeOr(lanelet::AttributeName::Type, "") == std::string("stop_line");
    };

    lanelet::Optional<lanelet::LineString3d> stop_line = lanelet_map_ptr_->lineStringLayer.nearestUntil(point2d, search_func);

    const auto stop_line_seg_start = Ogre::Vector3(stop_line.get().front().x(), stop_line.get().front().y(), 100);
    const auto stop_line_seg_end   = Ogre::Vector3(stop_line.get().back().x(), stop_line.get().back().y(), 100);


    const double lane_distance = ((lane_seg_start.x + lane_seg_end.x)/2.0 - (start_.x+end_.x)/2.0)
                                *((lane_seg_start.x + lane_seg_end.x)/2.0 - (start_.x+end_.x)/2.0)
                                +((lane_seg_start.y + lane_seg_end.y)/2.0 - (start_.y+end_.y)/2.0)
                                *((lane_seg_start.y + lane_seg_end.y)/2.0 - (start_.y+end_.y)/2.0);
    const double stop_line_distance = ((stop_line_seg_start.x + stop_line_seg_end.x)/2.0 - (start_.x+end_.x)/2.0)
                                     *((stop_line_seg_start.x + stop_line_seg_end.x)/2.0 - (start_.x+end_.x)/2.0)
                                     +((stop_line_seg_start.y + stop_line_seg_end.y)/2.0 - (start_.y+end_.y)/2.0)
                                     *((stop_line_seg_start.y + stop_line_seg_end.y)/2.0 - (start_.y+end_.y)/2.0);

    enum class LINE_TYPE{
      LANE,
      STOP_LINE
    };

    LINE_TYPE line_type;
    if (lane_distance < stop_line_distance) {
      seg_start_ = lane_seg_start;
      seg_end_ = lane_seg_end;
      line_type = LINE_TYPE::LANE;
    } else {
      seg_start_ = stop_line_seg_start;
      seg_end_ = stop_line_seg_end;
      line_type = LINE_TYPE::STOP_LINE;
    }

    // std::cout << seg_start_.x << " " << seg_start_.y << " " << seg_end_.x << " " << seg_end_.y << std::endl;


    line_lane_->clear();
    line_lane_->addPoint(seg_start_, color_lane_);
    line_lane_->addPoint(seg_end_, color_lane_);


    const double yaw_line = std::atan2(end_.y - start_.y, end_.x - start_.x);
    const double yaw_lane = std::atan2(seg_end_.y - seg_start_.y, seg_end_.x - seg_start_.x);
    const double yaw = std::remainder(yaw_line-yaw_lane, 2 * M_PI);


    const double distance_selfpose_to_line = autoware_utils::distance(
      autoware_utils::alt::Point2d(self_pose_.position.x, self_pose_.position.y), 
      autoware_utils::alt::Point2d(start_.x, start_.y),
      autoware_utils::alt::Point2d(end_.x, end_.y));

    const double distance_selfpose_to_lane = autoware_utils::distance(
      autoware_utils::alt::Point2d(self_pose_.position.x, self_pose_.position.y), 
      autoware_utils::alt::Point2d(seg_start_.x, seg_start_.y),
      autoware_utils::alt::Point2d(seg_end_.x, seg_end_.y));

    if (line_type == LINE_TYPE::LANE) {
     std::cout << "x_error:" << "none" << "[m], y_error: " << distance_selfpose_to_line - distance_selfpose_to_lane << "[m], yaw_error:  " << yaw/M_PI*180.0 << "[deg] " << std::endl;
    }
    else {
     std::cout << "x_error:" << distance_selfpose_to_line - distance_selfpose_to_lane << "[m], y_error: " << "none" << "[m], yaw_error:  " << yaw/M_PI*180.0 << "[deg] " << std::endl;
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

}  // namespace evaluate_position_error_using_line_rviz_plugin
}  // namespace autoware

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::evaluate_position_error_using_line_rviz_plugin::EvaluatePositionErrorUsingLineTool, rviz_common::Tool)
