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

#ifndef LANELET2_MAP_MERGER_HPP_
#define LANELET2_MAP_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::lanelet2_map_merger
{

class Lanelet2MapMerger
{
public:
  explicit Lanelet2MapMerger(const rclcpp::Logger & logger) : logger_(logger) {}

  void set_input_dir(const std::string & input_dir) { input_dir_ = input_dir; }
  void set_output(const std::string & output) { output_ = output; }
  void set_map_projector_info_path(const std::string & path) { map_projector_info_path_ = path; }

  void run();

private:
  std::vector<std::string> discover_osm_files(const std::string & input_dir) const;
  std::unique_ptr<lanelet::Projector> create_projector(
    const autoware_map_msgs::msg::MapProjectorInfo & projector_info) const;
  lanelet::LaneletMapPtr load_and_merge_maps(
    const std::vector<std::string> & osm_files,
    const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
    lanelet::Projector & projector);

  std::string input_dir_;
  std::string output_;
  std::string map_projector_info_path_;
  // Keep loaded source maps alive so references from merged primitives do not expire.
  std::vector<lanelet::LaneletMapPtr> loaded_maps_;
  rclcpp::Logger logger_;
};

}  // namespace autoware::lanelet2_map_merger

#endif  // LANELET2_MAP_MERGER_HPP_
