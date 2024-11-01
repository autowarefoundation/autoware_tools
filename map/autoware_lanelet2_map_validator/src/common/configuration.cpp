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

#include "lanelet2_map_validator/configuration.hpp"

namespace lanelet::autoware::validation
{
const Configuration & getConfiguration(lanelet::LaneletMapPtr lanelet_map_ptr)
{
  static Configuration config;
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  config.vehicle_routing_graph_ptr =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *traffic_rules);
  config.pedestrian_routing_graph_ptr =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *pedestrian_rules);
  return config;
}
}  // namespace lanelet::autoware::validation
