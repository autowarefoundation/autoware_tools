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

#include "lanelet2_map_validator/map_loader.hpp"

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lanelet::autoware::validation
{

namespace
{
namespace projector_names
{
constexpr const char * mgrs = "mgrs";
constexpr const char * transverse_mercator = "transverse_mercator";
constexpr const char * utm = "utm";
}  // namespace projector_names
}  // namespace

std::unique_ptr<lanelet::Projector> getProjector(
  const std::string & projector_type, const lanelet::GPSPoint & origin)

{
  if (projector_type == projector_names::mgrs) {
    return std::make_unique<lanelet::projection::MGRSProjector>();
  }
  if (projector_type == projector_names::transverse_mercator) {
    return std::make_unique<lanelet::projection::TransverseMercatorProjector>(
      lanelet::Origin{origin});
  }
  if (projector_type == projector_names::utm) {
    return std::make_unique<lanelet::projection::UtmProjector>(lanelet::Origin{origin});
  }
  return nullptr;
}

std::pair<lanelet::LaneletMapPtr, std::vector<lanelet::validation::DetectedIssues>>
loadAndValidateMap(
  const std::string & projector_type, const std::string & map_file,
  const lanelet::validation::ValidationConfig & val_config)
{
  std::vector<lanelet::validation::DetectedIssues> issues;
  lanelet::LaneletMapPtr map{nullptr};
  lanelet::validation::Strings errors;
  try {
    const auto projector = getProjector(projector_type, val_config.origin);
    if (!projector) {
      errors.push_back("No valid map projection type specified!");
    } else {
      map = lanelet::load(map_file, "autoware_validator_osm_handler", *projector, &errors);
    }
    if (!map) {
      errors.push_back("Failed to load map!");
    }
    issues.emplace_back("general", utils::transform(errors, [](auto & error) {
                          return lanelet::validation::Issue(
                            lanelet::validation::Severity::Error, error);
                        }));
  } catch (lanelet::LaneletError & err) {
    issues.emplace_back("general", utils::transform(errors, [](auto & error) {
                          return lanelet::validation::Issue(
                            lanelet::validation::Severity::Error, error);
                        }));
  }

  return {map, issues};
}

}  // namespace lanelet::autoware::validation
