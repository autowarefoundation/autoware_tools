// Copyright 2023 Autoware Foundation
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

#include "autoware_lanelet2_map_validator/validation.hpp"

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>

namespace lanelet
{
namespace autoware
{
namespace validation
{

std::unique_ptr<lanelet::Projector> getProjector(const MetaConfig & config)
{
  const auto & val_config = config.command_line_config.validationConfig;
  if (config.projector_type == projector_names::mgrs) {
    return std::make_unique<lanelet::projection::MGRSProjector>();
  } else if (config.projector_type == projector_names::transverse_mercator) {
    return std::make_unique<lanelet::projection::TransverseMercatorProjector>(
      lanelet::Origin{val_config.origin});
  } else if (config.projector_type == projector_names::utm) {
    return std::make_unique<lanelet::projection::UtmProjector>(lanelet::Origin{val_config.origin});
  } else {
    //std::cerr << "Set to default projector: MGRS projector" << std::endl;
    return std::make_unique<lanelet::projection::MGRSProjector>();
  }
}

std::vector<lanelet::validation::DetectedIssues> validateMap(const MetaConfig & config)
{
  const auto & cm_config = config.command_line_config;
  const auto & val_config = config.command_line_config.validationConfig;

  const auto & parse_filter = [](const std::string & str) {
    std::vector<std::regex> regexes;
    std::stringstream ss(str);

    while (ss.good()) {
      std::string substr;
      getline(ss, substr, ',');
      if (substr.empty()) {
        continue;
      }
      regexes.emplace_back(substr, std::regex::basic | std::regex::icase);
    }
    return regexes;
  };

  auto checks = parse_filter(val_config.checksFilter);

  std::vector<lanelet::validation::DetectedIssues> issues;
  lanelet::LaneletMapPtr map;
  lanelet::validation::Strings errors;
  try {
    const auto & projector = getProjector(config);
    map = lanelet::load(cm_config.mapFile, *projector, &errors);
    if (!errors.empty()) {
      std::cout << "!errors.empty()" << std::endl;
      issues.emplace_back("general", utils::transform(errors, [](auto & error) {
                            return lanelet::validation::Issue(
                              lanelet::validation::Severity::Error, error);
                          }));
    }
  } catch (lanelet::LaneletError & err) {
    std::cout << "catch" << std::endl;
    issues.emplace_back("general", utils::transform(errors, [](auto & error) {
                          return lanelet::validation::Issue(
                            lanelet::validation::Severity::Error, error);
                        }));
  }

  appendIssues(issues, lanelet::validation::validateMap(*map, val_config));
  return issues;
}

}  // namespace validation
}  // namespace autoware
}  // namespace lanelet
