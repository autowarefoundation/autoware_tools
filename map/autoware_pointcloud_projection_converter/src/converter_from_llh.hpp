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

#ifndef POINTCLOUD_PROJECTION_CONVERTER__CONVERTER_FROM_LLH_HPP
#define POINTCLOUD_PROJECTION_CONVERTER__CONVERTER_FROM_LLH_HPP

#include <utility>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/TransverseMercatorExact.hpp>
#include <iostream>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include "lat_lon_alt.hpp"

namespace autoware::pointcloud_projection_converter
{

class ConverterFromLLH {
public:
  ConverterFromLLH(const YAML::Node &config);
  pcl::PointXYZI convert(const LatLonAlt &xyz);

private:
  std::string projector_type_;
  std::pair<double, double> origin_xy_;
  double central_meridian_;
};

}  // namespace autoware::pointcloud_projection_converter

#endif // POINTCLOUD_PROJECTION_CONVERTER__CONVERTER_FROM_LLH_HPP
