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

#include "lanelet2_map_merger_node.hpp"

#include "lanelet2_map_merger.hpp"

#include <sstream>
#include <string>

namespace autoware::lanelet2_map_merger
{

Lanelet2MapMergerNode::Lanelet2MapMergerNode(const rclcpp::NodeOptions & node_options)
: Node("lanelet2_map_merger", node_options)
{
  const std::string input_lanelet2_map_dir =
    declare_parameter<std::string>("input_lanelet2_map_dir");
  const std::string output_lanelet2_map = declare_parameter<std::string>("output_lanelet2_map");
  const std::string map_projector_info_path =
    declare_parameter<std::string>("map_projector_info_path");

  std::ostringstream oss;
  oss << "\n########## Input Parameters ##########\n"
      << "\tinput_lanelet2_map_dir: " << input_lanelet2_map_dir << "\n"
      << "\toutput_lanelet2_map: " << output_lanelet2_map << "\n"
      << "\tmap_projector_info_path: " << map_projector_info_path << "\n"
      << "######################################";
  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

  Lanelet2MapMerger merger(get_logger());
  merger.set_input_dir(input_lanelet2_map_dir);
  merger.set_output(output_lanelet2_map);
  merger.set_map_projector_info_path(map_projector_info_path);

  merger.run();

  rclcpp::shutdown();
}

}  // namespace autoware::lanelet2_map_merger

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lanelet2_map_merger::Lanelet2MapMergerNode)
