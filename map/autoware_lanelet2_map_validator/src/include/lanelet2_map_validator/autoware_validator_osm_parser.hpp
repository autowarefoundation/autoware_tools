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

#ifndef LANELET2_MAP_VALIDATOR__AUTOWARE_VALIDATOR_OSM_PARSER_HPP_
#define LANELET2_MAP_VALIDATOR__AUTOWARE_VALIDATOR_OSM_PARSER_HPP_

#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <memory>
#include <string>

namespace lanelet::io_handlers
{
class AutowareValidatorOsmParser : public lanelet::io_handlers::OsmParser
{
public:
  std::unique_ptr<LaneletMap> parse(
    const std::string & filename, ErrorMessages & errors) const override;

  static constexpr const char * extension() { return ".osm"; }

  static constexpr const char * name() { return "autoware_validator_osm_handler"; }

  AutowareValidatorOsmParser(const lanelet::Projector & projector, const io::Configuration & config)
  : OsmParser(projector, config)
  {
  }
};
}  // namespace lanelet::io_handlers

namespace lanelet::osm::autoware
{
File read(pugi::xml_document & node, lanelet::osm::Errors * errors = nullptr);
}

#endif  // LANELET2_MAP_VALIDATOR__AUTOWARE_VALIDATOR_OSM_PARSER_HPP_
