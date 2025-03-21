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

void transform_maps(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map_ptr,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const Eigen::Affine3d & affine)
{
  {
    for (lanelet::Point3d & pt : lanelet_map_ptr->pointLayer) {
      Eigen::Vector3d eigen_pt(pt.x(), pt.y(), pt.z());
      auto transformed_pt = affine * eigen_pt;
      pt.x() = transformed_pt.x();
      pt.y() = transformed_pt.y();
      pt.z() = transformed_pt.z();
    }
  }

  {
    for (auto & pt : pcd_map_ptr->points) {
      Eigen::Vector3d eigen_pt(pt.x, pt.y, pt.z);
      auto transformed_pt = affine * eigen_pt;
      pt.x = static_cast<float>(transformed_pt.x());
      pt.y = static_cast<float>(transformed_pt.y());
      pt.z = static_cast<float>(transformed_pt.z());
    }
  }
}

Eigen::Affine3d create_affine_matrix_from_xyzrpy(
  const double x, const double y, const double z, const double roll, const double pitch,
  const double yaw)
{
  double roll_rad = roll * M_PI / 180.0;
  double pitch_rad = pitch * M_PI / 180.0;
  double yaw_rad = yaw * M_PI / 180.0;

  Eigen::Translation<double, 3> trans(x, y, z);
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
  return trans * rot;
}
}  // namespace autoware::lanelet2_map_utils

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("transform_maps");

  const auto llt_map_path = node->declare_parameter<std::string>("llt_map_path");
  const auto pcd_map_path = node->declare_parameter<std::string>("pcd_map_path");
  const auto llt_output_path = node->declare_parameter<std::string>("llt_output_path");
  const auto pcd_output_path = node->declare_parameter<std::string>("pcd_output_path");
  const auto x = node->declare_parameter<double>("x");
  const auto y = node->declare_parameter<double>("y");
  const auto z = node->declare_parameter<double>("z");
  const auto roll = node->declare_parameter<double>("roll");
  const auto pitch = node->declare_parameter<double>("pitch");
  const auto yaw = node->declare_parameter<double>("yaw");

  std::cout << "transforming maps with following parameters" << std::endl
            << "x " << x << std::endl
            << "y " << y << std::endl
            << "z " << z << std::endl
            << "roll " << roll << std::endl
            << "pitch " << pitch << std::endl
            << "yaw " << yaw << std::endl;

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (!autoware::lanelet2_map_utils::load_lanelet_map(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }
  if (!autoware::lanelet2_map_utils::load_pcd_map(pcd_map_path, pcd_map_ptr)) {
    return EXIT_FAILURE;
  }
  Eigen::Affine3d affine =
    autoware::lanelet2_map_utils::create_affine_matrix_from_xyzrpy(x, y, z, roll, pitch, yaw);

  const auto mgrs_grid =
    node->declare_parameter<std::string>("mgrs_grid", projector.getProjectedMGRSGrid());
  std::cout << "using mgrs grid: " << mgrs_grid << std::endl;

  autoware::lanelet2_map_utils::transform_maps(pcd_map_ptr, llt_map_ptr, affine);
  lanelet::write(llt_output_path, *llt_map_ptr, projector);
  pcl::io::savePCDFileBinary(pcd_output_path, *pcd_map_ptr);

  rclcpp::shutdown();

  return 0;
}
