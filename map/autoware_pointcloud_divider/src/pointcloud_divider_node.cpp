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

#include <pcl/point_types.h>

#include "include/pointcloud_divider_node.hpp"

#include <autoware/pointcloud_divider/pcd_divider.hpp>

namespace autoware::pointcloud_divider
{
    
PointCloudDivider::PointCloudDivider(const rclcpp::NodeOptions & node_options)
: Node("pointcloud_divider", node_options)
{
    // Load command parameters
    std::string input_pcd_or_dir = this->declare_parameter("input_pcd_or_dir", "");
    std::string output_dir = this->declare_parameter("output_pcd_dir", "");
    std::string file_prefix = this->declare_parameter("prefix", "");
    std::string config_file = this->declare_parameter("config_file", "");
    std::string point_type = this->declare_parameter("point_type", "point_xyzi");

    RCLCPP_INFO(get_logger(), "input_pcd_or_dir: %s", input_pcd_or_dir.c_str());
    RCLCPP_INFO(get_logger(), "output_dir: %s", output_dir.c_str());
    RCLCPP_INFO(get_logger(), "file_prefix: %s", file_prefix.c_str());
    RCLCPP_INFO(get_logger(), "config_file: %s", config_file.c_str());
    RCLCPP_INFO(get_logger(), "point_type: %s", point_type.c_str());

    if (point_type == "point_xyz") {
        autoware::pointcloud_divider::PCDDivider<pcl::PointXYZ> pcd_divider_exe(this->get_logger());

        pcd_divider_exe.setInput(input_pcd_or_dir);
        pcd_divider_exe.setOutputDir(output_dir);
        pcd_divider_exe.setPrefix(file_prefix);
        pcd_divider_exe.setConfig(config_file);

        pcd_divider_exe.run();
    } else if (point_type == "point_xyzi") {
        autoware::pointcloud_divider::PCDDivider<pcl::PointXYZI> pcd_divider_exe(this->get_logger());

        pcd_divider_exe.setInput(input_pcd_or_dir);
        pcd_divider_exe.setOutputDir(output_dir);
        pcd_divider_exe.setPrefix(file_prefix);
        pcd_divider_exe.setConfig(config_file);

        pcd_divider_exe.run();
    }

    rclcpp::shutdown();
}

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_divider::PointCloudDivider)
