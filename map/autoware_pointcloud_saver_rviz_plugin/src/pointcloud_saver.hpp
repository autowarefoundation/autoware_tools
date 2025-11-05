
#ifndef POINTCLOUD_SAVER_HPP_
#define POINTCLOUD_SAVER_HPP_

#include <memory>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order

#include <OgreVector.h>

#include <rclcpp/rclcpp.hpp>
#include "rviz_common/tool.hpp"

#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include <rviz_rendering/objects/billboard_line.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>

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
}

namespace autoware
{
namespace pointcloud_saver_rviz_plugin
{
  using PointType = pcl::PointXYZ;

  class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudSaver : public rviz_common::Tool
{
  Q_OBJECT

public:
  PointCloudSaver();

  void onInitialize() override;

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_ptr);

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:
  void updateLineColor();

private:
  void savePointCloud(std::vector<Ogre::Vector3> line_points);
  void transform_sensor_measurement(
    const std::string & source_frame,
    const std::string & target_frame,
    const std::shared_ptr<pcl::PointCloud<PointType>> & sensor_points_input_ptr,
    std::shared_ptr<pcl::PointCloud<PointType>> & sensor_points_output_ptr
  );

  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

  rviz_common::properties::ColorProperty * color_property_;

  std::shared_ptr<pcl::PointCloud<PointType>> sensor_points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sensor_points_;


  // lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::shared_ptr<rviz_rendering::BillboardLine> line_viz;

  Ogre::ColourValue color_;

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  QCursor std_cursor_;
  QCursor hit_cursor_;

};

}  // namespace pointcloud_saver_rviz_plugin
}  // namespace autoware

#endif  // POINTCLOUD_SAVER_HPP_