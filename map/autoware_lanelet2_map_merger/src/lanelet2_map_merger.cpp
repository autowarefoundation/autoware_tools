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

#include "lanelet2_map_merger.hpp"

#include "local_projector.hpp"

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>

#include <lanelet2_io/Io.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::lanelet2_map_merger
{

namespace
{

bool is_osm_file(const fs::path & path)
{
  if (fs::is_directory(path)) {
    return false;
  }
  const std::string ext = path.extension().string();
  return ext == ".osm" || ext == ".OSM";
}

}  // namespace

std::vector<std::string> Lanelet2MapMerger::discover_osm_files(const std::string & input_dir) const
{
  std::vector<std::string> osm_files;
  fs::path input_path(input_dir);

  if (!fs::exists(input_path) || !fs::is_directory(input_path)) {
    RCLCPP_ERROR(logger_, "Input is not a valid directory: %s", input_dir.c_str());
    return osm_files;
  }

  for (const auto & entry : fs::directory_iterator(input_path)) {
    if (is_osm_file(entry.path())) {
      osm_files.push_back(entry.path().string());
    }
  }
  std::sort(osm_files.begin(), osm_files.end());

  RCLCPP_INFO(logger_, "Found %zu .osm files in %s", osm_files.size(), input_dir.c_str());
  return osm_files;
}

std::unique_ptr<lanelet::Projector> Lanelet2MapMerger::create_projector(
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info) const
{
  if (projector_info.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    return std::make_unique<LocalProjector>();
  }
  return autoware::geography_utils::get_lanelet2_projector(projector_info);
}

lanelet::LaneletMapPtr Lanelet2MapMerger::load_and_merge_maps(
  const std::vector<std::string> & osm_files,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info, lanelet::Projector & projector)
{
  auto merged = std::make_shared<lanelet::LaneletMap>();

  const bool is_local =
    projector_info.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL;

  for (const auto & path : osm_files) {
    RCLCPP_INFO(logger_, "Loading %s", path.c_str());
    lanelet::ErrorMessages errors;
    lanelet::LaneletMapPtr map = lanelet::load(path, "autoware_osm_handler", projector, &errors);
    for (const auto & error : errors) {
      RCLCPP_ERROR(logger_, "Error loading %s: %s", path.c_str(), error.c_str());
    }
    if (!errors.empty() || !map) {
      return nullptr;
    }

    if (is_local) {
      for (lanelet::Point3d & point : map->pointLayer) {
        if (point.hasAttribute("local_x")) {
          point.x() = point.attribute("local_x").asDouble().value();
        }
        if (point.hasAttribute("local_y")) {
          point.y() = point.attribute("local_y").asDouble().value();
        }
      }
    }

    // Merge with deduplication of shared points (same id), matching
    // lanelet2_map_loader's merge_lanelet2_maps behavior.
    for (lanelet::Lanelet & llt : map->laneletLayer) {
      merged->add(llt);
    }
    for (lanelet::Area & area : map->areaLayer) {
      merged->add(area);
    }
    for (lanelet::RegulatoryElementPtr & reg : map->regulatoryElementLayer) {
      merged->add(reg);
    }
    for (lanelet::LineString3d & ls : map->lineStringLayer) {
      for (lanelet::Point3d & pt : ls) {
        if (merged->pointLayer.find(pt.id()) != merged->pointLayer.end()) {
          pt = merged->pointLayer.get(pt.id());
        }
      }
      merged->add(ls);
    }
    for (lanelet::Polygon3d & poly : map->polygonLayer) {
      merged->add(poly);
    }
    for (lanelet::Point3d & pt : map->pointLayer) {
      if (merged->pointLayer.find(pt.id()) == merged->pointLayer.end()) {
        merged->add(pt);
      }
    }

    // Keep the source map alive so weak references from regulatory elements
    // (e.g., parameters referencing other lanelets/linestrings) do not expire.
    loaded_maps_.push_back(map);
  }

  return merged;
}

void Lanelet2MapMerger::run()
{
  const auto osm_files = discover_osm_files(input_dir_);
  if (osm_files.empty()) {
    RCLCPP_ERROR(logger_, "No .osm files found under %s", input_dir_.c_str());
    return;
  }

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

  lanelet::LaneletMapPtr merged = load_and_merge_maps(osm_files, projector_info, *projector);
  if (!merged) {
    RCLCPP_ERROR(logger_, "Failed to load Lanelet2 maps from %s", input_dir_.c_str());
    return;
  }

  const fs::path out_path(output_);
  if (out_path.has_parent_path()) {
    fs::create_directories(out_path.parent_path());
  }
  if (fs::exists(out_path)) {
    fs::remove(out_path);
  }

  lanelet::write(output_, *merged, *projector);
  RCLCPP_INFO(logger_, "Saved merged map: %s", output_.c_str());
}

}  // namespace autoware::lanelet2_map_merger
