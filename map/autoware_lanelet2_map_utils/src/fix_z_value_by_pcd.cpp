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
#include <lanelet2_io/Io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <iostream>
#include <limits>
#include <string>
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

bool load_pcd_map(
  const std::string & pcd_map_path, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map_ptr)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_map_path, *pcd_map_ptr) == -1) {  //* load the file
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadPCDMap"), "Couldn't read file: " << pcd_map_path);
    return false;
  }
  std::cout << "Loaded " << pcd_map_ptr->width * pcd_map_ptr->height << " data points."
            << std::endl;
  return true;
}

double get_min_height_around_point(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map_ptr,
  const pcl::KdTreeFLANN<pcl::PointXYZ> & kdtree, const pcl::PointXYZ & search_pt,
  const double search_radius3d, const double search_radius2d)
{
  std::vector<int> point_idx_radius_search;
  std::vector<float> point_radius_squared_distance;
  if (
    kdtree.radiusSearch(
      search_pt, search_radius3d, point_idx_radius_search, point_radius_squared_distance) <= 0) {
    std::cout << "no points found within 3d radius " << search_radius3d << std::endl;
    return search_pt.z;
  }

  double min_height = std::numeric_limits<double>::max();
  bool found = false;

  for (auto pt_idx : point_idx_radius_search) {
    const auto pt = pcd_map_ptr->points.at(pt_idx);
    if (pt.z > min_height) {
      continue;
    }
    double distance2d = std::hypot(pt.x - search_pt.x, pt.y - search_pt.y);
    if (distance2d < search_radius2d) {
      found = true;
      min_height = pt.z;
    }
  }
  if (!found) {
    min_height = search_pt.z;
  }
  return min_height;
}

void adjust_height(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map_ptr,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  std::unordered_set<lanelet::Id> done;
  double search_radius2d = 0.5;
  double search_radius3d = 10;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pcd_map_ptr);

  for (lanelet::Lanelet & llt : lanelet_map_ptr->laneletLayer) {
    for (lanelet::Point3d & pt : llt.leftBound()) {
      if (done.find(pt.id()) != done.end()) {
        continue;
      }
      pcl::PointXYZ pcl_pt;
      pcl_pt.x = static_cast<float>(pt.x());
      pcl_pt.y = static_cast<float>(pt.y());
      pcl_pt.z = static_cast<float>(pt.z());
      double min_height =
        get_min_height_around_point(pcd_map_ptr, kdtree, pcl_pt, search_radius3d, search_radius2d);
      std::cout << "moving from " << pt.z() << " to " << min_height << std::endl;
      pt.z() = min_height;
      done.insert(pt.id());
    }
    for (lanelet::Point3d & pt : llt.rightBound()) {
      if (done.find(pt.id()) != done.end()) {
        continue;
      }
      pcl::PointXYZ pcl_pt;
      pcl_pt.x = static_cast<float>(pt.x());
      pcl_pt.y = static_cast<float>(pt.y());
      pcl_pt.z = static_cast<float>(pt.z());
      double min_height =
        get_min_height_around_point(pcd_map_ptr, kdtree, pcl_pt, search_radius3d, search_radius2d);
      std::cout << "moving from " << pt.z() << " to " << min_height << std::endl;
      pt.z() = min_height;
      done.insert(pt.id());
    }
  }
}
}  // namespace autoware::lanelet2_map_utils

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("lanelet_map_height_adjuster");

  const auto llt_map_path = node->declare_parameter<std::string>("llt_map_path");
  const auto pcd_map_path = node->declare_parameter<std::string>("pcd_map_path");
  const auto llt_output_path = node->declare_parameter<std::string>("llt_output_path");

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (!autoware::lanelet2_map_utils::load_lanelet_map(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }
  if (!autoware::lanelet2_map_utils::load_pcd_map(pcd_map_path, pcd_map_ptr)) {
    return EXIT_FAILURE;
  }

  autoware::lanelet2_map_utils::adjust_height(pcd_map_ptr, llt_map_ptr);
  lanelet::write(llt_output_path, *llt_map_ptr, projector);

  rclcpp::shutdown();

  return 0;
}
