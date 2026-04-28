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

#include "lanelet2_map_divider.hpp"

#include "local_projector.hpp"

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_io/Io.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::lanelet2_map_divider
{

namespace
{

bool is_osm_file(const fs::path & path)
{
  if (!fs::exists(path) || fs::is_directory(path)) {
    return false;
  }
  const std::string ext = path.extension().string();
  return ext == ".osm" || ext == ".OSM";
}

}  // namespace

std::unique_ptr<lanelet::Projector> Lanelet2MapDivider::create_projector(
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info) const
{
  if (projector_info.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    return std::make_unique<LocalProjector>();
  }
  return autoware::geography_utils::get_lanelet2_projector(projector_info);
}

lanelet::LaneletMapPtr Lanelet2MapDivider::load_map(
  const std::string & osm_file, const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
  lanelet::Projector & projector) const
{
  lanelet::ErrorMessages errors;
  lanelet::LaneletMapPtr loaded =
    lanelet::load(osm_file, "autoware_osm_handler", projector, &errors);
  for (const auto & error : errors) {
    RCLCPP_ERROR(logger_, "Error loading %s: %s", osm_file.c_str(), error.c_str());
  }
  if (!errors.empty() || !loaded) {
    return nullptr;
  }

  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    return loaded;
  }

  // LOCAL projection: override each point's x/y from its `local_x`/`local_y` tags,
  // then rebuild the map so the spatial index reflects the updated coordinates
  // (mutating points in place leaves the rtree stale and breaks layer search).
  for (lanelet::Point3d point : loaded->pointLayer) {
    if (point.hasAttribute("local_x")) {
      point.x() = point.attribute("local_x").asDouble().value();
    }
    if (point.hasAttribute("local_y")) {
      point.y() = point.attribute("local_y").asDouble().value();
    }
  }

  auto rebuilt = std::make_shared<lanelet::LaneletMap>();
  for (auto & llt : loaded->laneletLayer) rebuilt->add(llt);
  for (auto & area : loaded->areaLayer) rebuilt->add(area);
  for (auto & reg : loaded->regulatoryElementLayer) rebuilt->add(reg);
  for (auto & ls : loaded->lineStringLayer) rebuilt->add(ls);
  for (auto & poly : loaded->polygonLayer) rebuilt->add(poly);
  for (auto & pt : loaded->pointLayer) rebuilt->add(pt);
  return rebuilt;
}

void Lanelet2MapDivider::prepare_output_directory(const std::string & cell_dir) const
{
  if (fs::exists(output_dir_)) {
    fs::remove_all(output_dir_);
  }
  fs::create_directories(cell_dir);
}

std::string Lanelet2MapDivider::make_file_name(int gx, int gy) const
{
  std::string name;
  if (!prefix_.empty()) {
    name += prefix_ + "_";
  }
  name += std::to_string(gx) + "_" + std::to_string(gy) + ".osm";
  return name;
}

void Lanelet2MapDivider::divide_and_save(
  lanelet::LaneletMap & map, lanelet::Projector & projector, const std::string & cell_dir,
  std::vector<std::tuple<std::string, int, int>> & out_cells) const
{
  if (map.pointLayer.empty()) {
    RCLCPP_WARN(logger_, "Input map has no points; nothing to divide.");
    return;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const lanelet::ConstPoint3d & pt : map.pointLayer) {
    min_x = std::min(min_x, pt.x());
    min_y = std::min(min_y, pt.y());
    max_x = std::max(max_x, pt.x());
    max_y = std::max(max_y, pt.y());
  }

  const auto gx_start = static_cast<int>(std::floor(min_x / grid_size_x_) * grid_size_x_);
  const auto gy_start = static_cast<int>(std::floor(min_y / grid_size_y_) * grid_size_y_);
  const auto gx_end = static_cast<int>(std::floor(max_x / grid_size_x_) * grid_size_x_);
  const auto gy_end = static_cast<int>(std::floor(max_y / grid_size_y_) * grid_size_y_);

  RCLCPP_INFO(
    logger_, "Map bounding box: [%.3f, %.3f] to [%.3f, %.3f]", min_x, min_y, max_x, max_y);
  RCLCPP_INFO(
    logger_, "Grid range (coord-aligned): x=[%d, %d], y=[%d, %d]", gx_start, gx_end, gy_start,
    gy_end);

  const auto step_x = static_cast<int>(grid_size_x_);
  const auto step_y = static_cast<int>(grid_size_y_);

  for (int gx = gx_start; gx <= gx_end; gx += step_x) {
    for (int gy = gy_start; gy <= gy_end; gy += step_y) {
      const lanelet::BoundingBox2d cell_bbox(
        lanelet::BasicPoint2d(gx, gy), lanelet::BasicPoint2d(gx + grid_size_x_, gy + grid_size_y_));

      const auto lanelets = map.laneletLayer.search(cell_bbox);
      const auto areas = map.areaLayer.search(cell_bbox);
      const auto linestrings = map.lineStringLayer.search(cell_bbox);
      const auto points = map.pointLayer.search(cell_bbox);

      if (lanelets.empty() && areas.empty() && linestrings.empty() && points.empty()) {
        continue;
      }

      lanelet::LaneletMapPtr cell_map(new lanelet::LaneletMap);
      for (const auto & llt : lanelets) {
        cell_map->add(llt);
      }
      for (const auto & area : areas) {
        cell_map->add(area);
      }
      for (const auto & ls : linestrings) {
        cell_map->add(ls);
      }
      for (const auto & pt : points) {
        cell_map->add(pt);
      }

      const std::string file_name = make_file_name(gx, gy);
      const std::string out_path = cell_dir + "/" + file_name;
      lanelet::write(out_path, *cell_map, projector);
      RCLCPP_INFO(
        logger_, "Saved cell [%d, %d] -> %s (lanelets=%zu, areas=%zu, linestrings=%zu, points=%zu)",
        gx, gy, out_path.c_str(), lanelets.size(), areas.size(), linestrings.size(), points.size());

      out_cells.emplace_back(file_name, gx, gy);
    }
  }
}

void Lanelet2MapDivider::write_metadata(
  const std::string & yaml_path, const std::vector<std::tuple<std::string, int, int>> & cells) const
{
  std::ofstream file(yaml_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open metadata file: %s", yaml_path.c_str());
    return;
  }

  file << "x_resolution: " << grid_size_x_ << "\n";
  file << "y_resolution: " << grid_size_y_ << "\n";
  for (const auto & cell : cells) {
    const auto & [file_name, gx, gy] = cell;
    file << file_name << ": [" << gx << ", " << gy << "]\n";
  }
  RCLCPP_INFO(logger_, "Saved metadata: %s", yaml_path.c_str());
}

void Lanelet2MapDivider::run()
{
  // Grid sizes below 1.0 truncate to a step of 0 in divide_and_save's int-cast loop,
  // which would spin forever.
  if (grid_size_x_ < 1.0 || grid_size_y_ < 1.0) {
    RCLCPP_ERROR(
      logger_, "Grid size must be >= 1.0 (got x=%f, y=%f).", grid_size_x_, grid_size_y_);
    return;
  }

  if (!is_osm_file(input_lanelet2_map_)) {
    RCLCPP_ERROR(
      logger_,
      "Input must be a single .osm file: %s. "
      "If your map is split across multiple files, merge them first with "
      "autoware_lanelet2_map_merger.",
      input_lanelet2_map_.c_str());
    return;
  }
  RCLCPP_INFO(logger_, "Input Lanelet2 map file: %s", input_lanelet2_map_.c_str());

  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  try {
    projector_info = autoware::map_projection_loader::load_info_from_yaml(map_projector_info_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to load map projector info from %s: %s", map_projector_info_path_.c_str(),
      e.what());
    return;
  }
  RCLCPP_INFO(logger_, "Projector type: %s", projector_info.projector_type.c_str());

  std::unique_ptr<lanelet::Projector> projector;
  try {
    projector = create_projector(projector_info);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to create projector: %s", e.what());
    return;
  }

  lanelet::LaneletMapPtr map = load_map(input_lanelet2_map_, projector_info, *projector);
  if (!map) {
    RCLCPP_ERROR(logger_, "Failed to load Lanelet2 map: %s", input_lanelet2_map_.c_str());
    return;
  }

  const std::string cell_dir = output_dir_ + "/lanelet2_map.osm";
  prepare_output_directory(cell_dir);

  std::vector<std::tuple<std::string, int, int>> cells;
  divide_and_save(*map, *projector, cell_dir, cells);

  const std::string yaml_path = output_dir_ + "/lanelet2_map_metadata.yaml";
  write_metadata(yaml_path, cells);

  RCLCPP_INFO(logger_, "Saved %zu cells to %s", cells.size(), cell_dir.c_str());
}

}  // namespace autoware::lanelet2_map_divider
