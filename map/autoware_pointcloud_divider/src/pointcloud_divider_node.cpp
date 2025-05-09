// Copyright 2024 Autoware Foundation
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

#include "include/pointcloud_divider_node.hpp"

#include <autoware/pointcloud_divider/pcd_divider.hpp>

#include <pcl/point_types.h>

#include <string>

namespace autoware::pointcloud_divider
{

PointCloudDivider::PointCloudDivider(const rclcpp::NodeOptions & node_options)
: Node("pointcloud_divider", node_options)
{
  // Load command parameters
  bool use_large_grid = declare_parameter<bool>("use_large_grid", false);
  float leaf_size = declare_parameter<float>("leaf_size");
  float grid_size_x = declare_parameter<float>("grid_size_x");
  float grid_size_y = declare_parameter<float>("grid_size_y");
  std::string input_pcd_or_dir = declare_parameter<std::string>("input_pcd_or_dir");
  std::string output_pcd_dir = declare_parameter<std::string>("output_pcd_dir");
  std::string file_prefix = declare_parameter<std::string>("prefix");
  std::string point_type = declare_parameter<std::string>("point_type");
  bool meta_gen = declare_parameter<bool>("metadata_generate", false);
  // Enter a new line and clear it
  // This is to get rid of the prefix of RCLCPP_INFO
  std::string line_breaker(102, ' ');

  line_breaker[0] = '\n';
  line_breaker[1] = line_breaker[101] = '\r';

  std::ostringstream param_display;

  param_display << line_breaker << line_breaker << "########## Input Parameters ##########"
                << line_breaker;

  if (use_large_grid) {
    param_display << "\tuse_large_grid: True" << line_breaker;
  } else {
    param_display << "\tuse_large_grid: False" << line_breaker;
  }

  param_display << "\tleaf_size: " << leaf_size << line_breaker;
  param_display << "\tgrid_size: " << grid_size_x << ", " << grid_size_y << line_breaker;
  param_display << "\tinput_pcd_or_dir: " << input_pcd_or_dir << line_breaker;
  param_display << "\toutput_pcd_dir: " << output_pcd_dir << line_breaker;
  param_display << "\tfile_prefix: " << file_prefix << line_breaker;
  param_display << "\tpoint_type: " << point_type << line_breaker;
  param_display << "######################################" << line_breaker;

  RCLCPP_INFO(get_logger(), "%s", param_display.str().c_str());

  if (point_type == "point_xyz") {
    autoware::pointcloud_divider::PCDDivider<pcl::PointXYZ> pcd_divider_exe(get_logger());

    pcd_divider_exe.setLargeGridMode(use_large_grid);
    pcd_divider_exe.setLeafSize(leaf_size);
    pcd_divider_exe.setGridSize(grid_size_x, grid_size_y);
    pcd_divider_exe.setInput(input_pcd_or_dir);
    pcd_divider_exe.setOutputDir(output_pcd_dir);
    pcd_divider_exe.setPrefix(file_prefix);

    pcd_divider_exe.run(meta_gen);
  } else if (point_type == "point_xyzi") {
    autoware::pointcloud_divider::PCDDivider<pcl::PointXYZI> pcd_divider_exe(get_logger());

    pcd_divider_exe.setLargeGridMode(use_large_grid);
    pcd_divider_exe.setLeafSize(leaf_size);
    pcd_divider_exe.setGridSize(grid_size_x, grid_size_y);
    pcd_divider_exe.setInput(input_pcd_or_dir);
    pcd_divider_exe.setOutputDir(output_pcd_dir);
    pcd_divider_exe.setPrefix(file_prefix);

    pcd_divider_exe.run(meta_gen);
  }

  rclcpp::shutdown();
}

}  // namespace autoware::pointcloud_divider

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_divider::PointCloudDivider)
