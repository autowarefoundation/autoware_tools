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

namespace autoware::pointcloud_projection_converter
{

ConverterFromLLH::ConverterFromLLH(const YAML::Node & config)
{
  projector_type_ = config["projector_type"].as<std::string>();

  if (projector_type_ == "TransverseMercator") {
    central_meridian_ = config["map_origin"]["longitude"].as<double>();

    // Calculate origin in Transverse Mercator coordinate
    const GeographicLib::TransverseMercatorExact & proj =
      GeographicLib::TransverseMercatorExact::UTM();
    double x, y;
    proj.Forward(
      central_meridian_, config["map_origin"]["latitude"].as<double>(),
      config["map_origin"]["longitude"].as<double>(), x, y);
    origin_xy_ = std::pair<double, double>(x, y);
  }
}

pcl::PointXYZI ConverterFromLLH::convert(const LatLonAlt & llh)
{
  pcl::PointXYZI xyz;
  if (projector_type_ == "TransverseMercator") {
    const GeographicLib::TransverseMercatorExact & proj =
      GeographicLib::TransverseMercatorExact::UTM();

    // Variables to hold the results
    double x, y;

    // Convert to transverse mercator coordinates
    proj.Forward(central_meridian_, llh.lat, llh.lon, x, y);
    xyz.x = x - origin_xy_.first;
    xyz.y = y - origin_xy_.second;
    xyz.z = llh.alt;

  } else {
    std::cerr << "Only conversion to "
                 "TransverseMercator is supported currently.\n";
    std::cerr << "Not supported projector type: " << projector_type_ << std::endl;
    throw std::runtime_error("");
  }
  return xyz;
}

}  // namespace autoware::pointcloud_projection_converter
