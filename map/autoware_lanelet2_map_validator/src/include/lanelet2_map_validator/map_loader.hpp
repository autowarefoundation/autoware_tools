// Copyright 2024 Autoware Foundation
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

#ifndef LANELET2_MAP_VALIDATOR__MAP_LOADER_HPP_  // NOLINT
#define LANELET2_MAP_VALIDATOR__MAP_LOADER_HPP_  // NOLINT

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_validation/Validation.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lanelet::autoware::validation
{

std::pair<lanelet::LaneletMapPtr, std::vector<lanelet::validation::DetectedIssues>>
loadAndValidateMapLoad(
  const std::string & projector_type, const std::string & map_file,
  const lanelet::validation::ValidationConfig & val_config);

}  // namespace lanelet::autoware::validation

#endif  // LANELET2_MAP_VALIDATOR__MAP_LOADER_HPP_
