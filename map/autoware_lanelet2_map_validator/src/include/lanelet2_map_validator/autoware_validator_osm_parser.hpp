/*
 * Copyright (c) 2018
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * All rights reserved.
 *
 * Modifications by [Taiki Yamada/Autoware Foundation], 2025
 *
 * This software is licensed under the BSD-3-Clause license.
 * See the LICENSE file for details.
 */

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
