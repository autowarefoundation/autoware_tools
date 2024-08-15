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

#include "include/pointcloud_merger_node.hpp"

#include <autoware/pointcloud_merger/pcd_merger.hpp>

namespace autoware::pointcloud_merger
{
    
PointCloudMerger::PointCloudMerger(const rclcpp::NodeOptions & node_options)
: Node("pointcloud_merger", node_options)
{
    // Load command parameters
    std::string input_pcd_dir = this->declare_parameter("input_pcd_dir", "");
    std::string output_pcd = this->declare_parameter("output_pcd", "");
    std::string config_file = this->declare_parameter("config_file", "");
    std::string point_type = this->declare_parameter("point_type", "point_xyzi");

    if (point_type == "point_xyz") {
        autoware::pointcloud_merger::PCDMerger<pcl::PointXYZ> pcd_merger_exe(this->get_logger());

        pcd_merger_exe.setInput(input_pcd_dir);
        pcd_merger_exe.setOutput(output_pcd);
        pcd_merger_exe.setConfig(config_file);

        pcd_merger_exe.run();
    } else if (point_type == "point_xyzi") {
        autoware::pointcloud_merger::PCDMerger<pcl::PointXYZI> pcd_merger_exe(this->get_logger());

        pcd_merger_exe.setInput(input_pcd_dir);
        pcd_merger_exe.setOutput(output_pcd);
        pcd_merger_exe.setConfig(config_file);

        pcd_merger_exe.run();
    }

    rclcpp::shutdown();
}

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_merger::PointCloudMerger)
