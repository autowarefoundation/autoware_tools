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

#ifndef LANELET2_MAP_DIVIDER_HPP_
#define LANELET2_MAP_DIVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::lanelet2_map_divider
{

class Lanelet2MapDivider
{
public:
  explicit Lanelet2MapDivider(const rclcpp::Logger & logger) : logger_(logger) {}

  void set_input(const std::string & input_lanelet2_map)
  {
    input_lanelet2_map_ = input_lanelet2_map;
  }
  void set_output_dir(const std::string & output_dir) { output_dir_ = output_dir; }
  void set_prefix(const std::string & prefix) { prefix_ = prefix; }
  void set_grid_size(double grid_size_x, double grid_size_y)
  {
    grid_size_x_ = grid_size_x;
    grid_size_y_ = grid_size_y;
  }
  void set_map_projector_info_path(const std::string & path) { map_projector_info_path_ = path; }

  void run();

private:
  std::unique_ptr<lanelet::Projector> create_projector(
    const autoware_map_msgs::msg::MapProjectorInfo & projector_info) const;
  lanelet::LaneletMapPtr load_map(
    const std::string & osm_file, const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
    lanelet::Projector & projector) const;
  std::string make_file_name(int gx, int gy) const;
  void divide_and_save(
    lanelet::LaneletMap & map, lanelet::Projector & projector, const std::string & cell_dir,
    std::vector<std::tuple<std::string, int, int>> & out_cells) const;
  void write_metadata(
    const std::string & yaml_path,
    const std::vector<std::tuple<std::string, int, int>> & cells) const;

  std::string input_lanelet2_map_;
  std::string output_dir_;
  std::string prefix_;
  std::string map_projector_info_path_;
  double grid_size_x_ = 100.0;
  double grid_size_y_ = 100.0;
  rclcpp::Logger logger_;
};

}  // namespace autoware::lanelet2_map_divider

#endif  // LANELET2_MAP_DIVIDER_HPP_
