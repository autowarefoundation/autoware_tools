// Copyright 2025 Autoware Foundation
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

// The original code was written by Koji Minoda

#include "converter_from_llh.hpp"
#include "converter_to_llh.hpp"
#include "lat_lon_alt.hpp"

#include <GeographicLib/MGRS.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr << "Usage: ros2 run autoware_pointcloud_projection_converter "
                 "pointcloud_projection_converter input_pcd output_pcd "
                 "input_yaml output_yaml"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Load point cloud data from file
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) == -1) {
    std::cerr << "Couldn't read file " << argv[1] << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Parse YAML configuration files
  YAML::Node input_config = YAML::LoadFile(argv[3]);
  YAML::Node output_config = YAML::LoadFile(argv[4]);

  // Define converters
  autoware::pointcloud_projection_converter::ConverterToLLH to_llh(input_config);
  autoware::pointcloud_projection_converter::ConverterFromLLH from_llh(output_config);

  // Convert points
  const size_t n_points = cloud->points.size();

#pragma omp parallel for
  for (size_t i = 0; i < n_points; ++i) {
    auto & point = cloud->points[i];
    autoware::pointcloud_projection_converter::LatLonAlt llh = to_llh.convert(point);
    point = from_llh.convert(llh);
  }

  // Save converted point cloud to file
  pcl::io::savePCDFileBinary(argv[2], *cloud);

  std::cout << "Point cloud projection conversion completed successfully" << std::endl;

  return 0;
}
