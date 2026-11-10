// Copyright 2026 Autoware Foundation
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

#include "lanelet2_map_divider_node.hpp"

#include "lanelet2_map_divider.hpp"

#include <sstream>
#include <string>

namespace autoware::lanelet2_map_divider
{

Lanelet2MapDividerNode::Lanelet2MapDividerNode(const rclcpp::NodeOptions & node_options)
: Node("lanelet2_map_divider", node_options)
{
  const double grid_size_x = declare_parameter<double>("grid_size_x");
  const double grid_size_y = declare_parameter<double>("grid_size_y");
  const std::string input_lanelet2_map = declare_parameter<std::string>("input_lanelet2_map");
  const std::string output_lanelet2_map_dir =
    declare_parameter<std::string>("output_lanelet2_map_dir");
  const std::string map_projector_info_path =
    declare_parameter<std::string>("map_projector_info_path");
  const std::string prefix = declare_parameter<std::string>("prefix");

  std::ostringstream oss;
  oss << "\n########## Input Parameters ##########\n"
      << "\tgrid_size_x: " << grid_size_x << "\n"
      << "\tgrid_size_y: " << grid_size_y << "\n"
      << "\tinput_lanelet2_map: " << input_lanelet2_map << "\n"
      << "\toutput_lanelet2_map_dir: " << output_lanelet2_map_dir << "\n"
      << "\tmap_projector_info_path: " << map_projector_info_path << "\n"
      << "\tprefix: " << prefix << "\n"
      << "######################################";
  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

  Lanelet2MapDivider divider(get_logger());
  divider.set_grid_size(grid_size_x, grid_size_y);
  divider.set_input(input_lanelet2_map);
  divider.set_output_dir(output_lanelet2_map_dir);
  divider.set_map_projector_info_path(map_projector_info_path);
  divider.set_prefix(prefix);

  divider.run();

  rclcpp::shutdown();
}

}  // namespace autoware::lanelet2_map_divider

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lanelet2_map_divider::Lanelet2MapDividerNode)
