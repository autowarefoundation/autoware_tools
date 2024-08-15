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

#include "include/pointcloud_merger_node.hpp"

#include <autoware/pointcloud_merger/pcd_merger.hpp>

#include <pcl/point_types.h>

namespace autoware::pointcloud_merger
{

PointCloudMerger::PointCloudMerger(const rclcpp::NodeOptions & node_options)
: Node("pointcloud_merger", node_options)
{
  // Load command parameters
  float leaf_size = declare_parameter<float>("leaf_size");
  std::string input_pcd_dir = declare_parameter<std::string>("input_pcd_dir");
  std::string output_pcd = declare_parameter<std::string>("output_pcd");
  std::string point_type = declare_parameter<std::string>("point_type");

  // Enter a new line and clear it
  // This is to get rid of the prefix of RCLCPP_INFO
  std::string line_breaker(102, ' ');

  line_breaker[0] = '\n';
  line_breaker[1] = line_breaker[101] = '\r';

  std::ostringstream param_display;

  param_display << line_breaker << line_breaker << "########## Input Parameters ##########"
                << line_breaker;

  param_display << "\tleaf_size: " << leaf_size << line_breaker;
  param_display << "\tinput_pcd_dir: " << input_pcd_dir << line_breaker;
  param_display << "\toutput_pcd: " << output_pcd << line_breaker;
  param_display << "\tpoint_type: " << point_type << line_breaker;
  param_display << "######################################" << line_breaker;

  RCLCPP_INFO(get_logger(), "%s", param_display.str().c_str());

  if (point_type == "point_xyz") {
    autoware::pointcloud_merger::PCDMerger<pcl::PointXYZ> pcd_merger_exe(get_logger());

    pcd_merger_exe.setLeafSize(leaf_size);
    pcd_merger_exe.setInput(input_pcd_dir);
    pcd_merger_exe.setOutput(output_pcd);

    pcd_merger_exe.run();
  } else if (point_type == "point_xyzi") {
    autoware::pointcloud_merger::PCDMerger<pcl::PointXYZI> pcd_merger_exe(get_logger());

    pcd_merger_exe.setLeafSize(leaf_size);
    pcd_merger_exe.setInput(input_pcd_dir);
    pcd_merger_exe.setOutput(output_pcd);

    pcd_merger_exe.run();
  }

  rclcpp::shutdown();
}

}  // namespace autoware::pointcloud_merger

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_merger::PointCloudMerger)
