// Copyright 2025 Autoware Foundation
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

#include "pointcloud_saver.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/line.hpp"

#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware
{

namespace pointcloud_saver_rviz_plugin
{

PointCloudSaver::PointCloudSaver() : tf2_listener_(tf2_buffer_)
{
  // shortcut_key_ = 'n';

  color_property_ = new rviz_common::properties::ColorProperty(
    "Line color", Qt::darkYellow, "The topic on which to publish points.", getPropertyContainer(),
    SLOT(updateLineColor()), this);
}

void PointCloudSaver::onInitialize()
{
  line_viz = std::make_shared<rviz_rendering::BillboardLine>(context_->getSceneManager());
  updateLineColor();

  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");

  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  sub_sensor_points_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/concatenated/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&PointCloudSaver::onPointCloud, this, std::placeholders::_1));
}

void PointCloudSaver::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_ptr)
{
  sensor_points_ = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*msg_ptr, *sensor_points_);
}

void PointCloudSaver::activate()
{
}

void PointCloudSaver::deactivate()
{
}

int PointCloudSaver::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (sensor_points_ == nullptr) {
    return 0;
  }

  auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  Ogre::Vector3 pos = point_projection_on_xy_plane.second;

  static bool drawing = false;
  static std::vector<Ogre::Vector3> line_points;

  if (event.leftDown() && !drawing) {
    line_points.clear();
  }

  if (event.leftDown()) {
    drawing = true;
    line_points.push_back(pos);

    line_viz->clear();
    for (const auto & line_point : line_points) {
      line_viz->addPoint(line_point, color_);
    }
  }

  if (event.type == QEvent::MouseMove && drawing) {
    line_viz->clear();
    for (const auto & line_point : line_points) {
      line_viz->addPoint(line_point, color_);
    }
    line_viz->addPoint(pos, color_);
  }

  if (event.rightDown() && drawing) {
    line_viz->clear();
    for (const auto & line_point : line_points) {
      line_viz->addPoint(line_point, color_);
    }
    line_viz->addPoint(line_points.front(), color_);

    drawing = false;
    savePointCloud(line_points);
    return Render;
  }

  return 0;
}

struct Point
{
  double x;
  double y;
};

bool pointOnSegment(const Point & a, const Point & b, const Point & p, double eps = 1e-12)
{
  double cross = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y);
  if (std::fabs(cross) > eps) return false;
  double dot = (p.x - a.x) * (p.x - b.x) + (p.y - a.y) * (p.y - b.y);
  return dot <= eps;
}

bool pointInPolygon(const std::vector<Point> & poly, const Point & p, bool includeBoundary = true)
{
  int n = static_cast<int>(poly.size());
  if (n < 3) return false;
  if (includeBoundary) {
    for (int i = 0; i < n; ++i) {
      const Point & a = poly[i];
      const Point & b = poly[(i + 1) % n];
      if (pointOnSegment(a, b, p)) return true;
    }
  }

  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const Point & vi = poly[i];
    const Point & vj = poly[j];
    bool intersect = ((vi.y > p.y) != (vj.y > p.y));
    if (intersect) {
      double xIntersect = vj.x + (vi.x - vj.x) * ((p.y - vj.y) / (vi.y - vj.y));
      if (p.x < xIntersect) inside = !inside;
    }
  }
  return inside;
}

void PointCloudSaver::savePointCloud(std::vector<Ogre::Vector3> line_points)
{
  std::vector<Point> poly;
  for (const auto & line_point : line_points) {
    poly.push_back(Point{static_cast<double>(line_point.x), static_cast<double>(line_point.y)});
  }

  auto sensor_points_in_fixed_frame = std::make_shared<pcl::PointCloud<PointType>>();
  try {
    transform_sensor_measurement(
      sensor_points_->header.frame_id, context_->getFixedFrame().toStdString(), sensor_points_,
      sensor_points_in_fixed_frame);
  } catch (const std::exception & ex) {
    std::cerr << "[PointCloudSaver::savePointCloud] Transform error: " << ex.what()
              << ". Please publish TF '" << sensor_points_->header.frame_id << "' to '"
              << context_->getFixedFrame().toStdString() << "'" << std::endl;
    return;
  }

  pcl::PointCloud<PointType> save_points;
  for (const auto & sensor_point : sensor_points_in_fixed_frame->points) {
    bool inside = pointInPolygon(poly, Point{sensor_point.x, sensor_point.y});
    if (inside) {
      save_points.push_back(sensor_point);
    }
  }

  std::cout << "save_points.points.size() " << save_points.points.size() << std::endl;

  if (!save_points.points.empty()) {
    pcl::io::savePCDFileBinary("saved_pointcloud.pcd", save_points);
  }
}

Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose;
  tf2::fromMsg(ros_pose, eigen_pose);
  return eigen_pose;
}

Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose_affine = pose_to_affine3d(ros_pose);
  Eigen::Matrix4f eigen_pose_matrix = eigen_pose_affine.matrix().cast<float>();
  return eigen_pose_matrix;
}

void PointCloudSaver::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const std::shared_ptr<pcl::PointCloud<PointType>> & sensor_points_input_ptr,
  std::shared_ptr<pcl::PointCloud<PointType>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  // May throw tf2::TransformException if transform is not available
  geometry_msgs::msg::TransformStamped transform =
    tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware_utils::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  autoware_utils::transform_pointcloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void PointCloudSaver::updateLineColor()
{
  color_ = rviz_common::properties::qtToOgre(color_property_->getColor());
  // line_->setColor(color);
  color_ = Ogre::ColourValue(0, 1, 0, 1);

  line_viz->setLineWidth(0.3);
}

}  // namespace pointcloud_saver_rviz_plugin
}  // namespace autoware

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::pointcloud_saver_rviz_plugin::PointCloudSaver, rviz_common::Tool)
