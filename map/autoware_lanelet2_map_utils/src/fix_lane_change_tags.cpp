// Copyright 2020 TIER IV, Inc.
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

#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <iostream>
#include <unordered_set>
#include <vector>

namespace autoware::lanelet2_map_utils
{
bool load_lanelet_map(
  const std::string & llt_map_path, lanelet::LaneletMapPtr & lanelet_map_ptr,
  lanelet::Projector & projector)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet_map_ptr = lanelet::load(llt_map_path, "autoware_osm_handler", projector, &errors);

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadLaneletMap"), error);
  }
  if (!errors.empty()) {
    return false;
  }
  std::cout << "Loaded Lanelet2 map" << std::endl;
  return true;
}

lanelet::Lanelets convert_to_vector(const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::Lanelets lanelets;
  std::copy(
    lanelet_map_ptr->laneletLayer.begin(), lanelet_map_ptr->laneletLayer.end(),
    std::back_inserter(lanelets));
  return lanelets;
}
void fix_tags(lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  auto lanelets = convert_to_vector(lanelet_map_ptr);
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
    lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphUPtr routing_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *traffic_rules);

  for (auto & llt : lanelets) {
    if (!routing_graph->conflicting(llt).empty()) {
      continue;
    }
    llt.attributes().erase("turn_direction");
    if (!!routing_graph->adjacentRight(llt)) {
      llt.rightBound().attributes()["lane_change"] = "yes";
    }
    if (!!routing_graph->adjacentLeft(llt)) {
      llt.leftBound().attributes()["lane_change"] = "yes";
    }
  }
}
}  // namespace autoware::lanelet2_map_utils

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("fix_lane_change_tags");

  const auto llt_map_path = node->declare_parameter<std::string>("llt_map_path");
  const auto output_path = node->declare_parameter<std::string>("output_path");

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  if (!autoware::lanelet2_map_utils::load_lanelet_map(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }

  autoware::lanelet2_map_utils::fix_tags(llt_map_ptr);
  lanelet::write(output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
