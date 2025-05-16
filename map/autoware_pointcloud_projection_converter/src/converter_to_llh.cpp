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

#include "converter_to_llh.hpp"

#include <iomanip>
#include <iostream>

namespace autoware::pointcloud_projection_converter
{

ConverterToLLH::ConverterToLLH(const YAML::Node & config)
{
  projector_type_ = config["projector_type"].as<std::string>();
  if (projector_type_ == "MGRS") {
    mgrs_grid_ = config["mgrs_grid"].as<std::string>();
  }
}

LatLonAlt ConverterToLLH::convert(const pcl::PointXYZI & xyz)
{
  LatLonAlt llh;
  if (projector_type_ == "MGRS") {
    try {
      int zone;
      bool northp;
      double mgrs_base_x, mgrs_base_y;
      int prec = 8;
      bool longpath = false;
      GeographicLib::MGRS::Reverse(
        mgrs_grid_, zone, northp, mgrs_base_x, mgrs_base_y, prec, longpath);

      // Convert UTM to LLH
      GeographicLib::UTMUPS::Reverse(
        zone, northp, xyz.x + mgrs_base_x, xyz.y + mgrs_base_y, llh.lat, llh.lon);

      llh.alt = xyz.z;
    } catch (const std::exception & e) {
      std::cerr << "Error: Could not convert from MGRS to UTM: " << e.what() << std::endl;
      return LatLonAlt();
    }
  }
  return llh;
}

}  // namespace autoware::pointcloud_projection_converter
