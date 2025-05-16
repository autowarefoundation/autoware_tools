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

#ifndef POINTCLOUD_PROJECTION_CONVERTER__LAT_LON_ALT_HPP
#define POINTCLOUD_PROJECTION_CONVERTER__LAT_LON_ALT_HPP

namespace autoware::pointcloud_projection_converter
{

struct LatLonAlt {
  double lat;
  double lon;
  double alt;
};

}  // namespace autoware::pointcloud_projection_converter

#endif // POINTCLOUD_PROJECTION_CONVERTER__LAT_LON_ALT_HPP
