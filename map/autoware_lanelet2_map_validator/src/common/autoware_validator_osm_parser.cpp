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

#include "lanelet2_map_validator/autoware_validator_osm_parser.hpp"

#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace lanelet::io_handlers
{
using Errors = std::vector<std::string>;

namespace
{
template <typename MapT>
void registerIds(const MapT & map)
{
  if (!map.empty()) {
    utils::registerId(map.rbegin()->first);
  }
}

void testAndPrintLocaleWarning(ErrorMessages & errors)
{
  auto * decimalPoint = std::localeconv()->decimal_point;
  if (decimalPoint == nullptr || *decimalPoint != '.') {
    std::stringstream ss;
    ss << "Warning: Current decimal point of the C locale is set to \""
       << (decimalPoint == nullptr ? ' ' : *decimalPoint)
       << "\". The loaded map will have wrong coordinates!\n";
    errors.emplace_back(ss.str());
    std::cerr << errors.back();
  }
}

Errors buildErrorMessage(const std::string & errorIntro, const Errors & errors)
{
  if (errors.empty()) {
    return {};
  }
  Errors message{errorIntro};
  message.reserve(errors.size() + 1);
  for (const auto & error : errors) {
    message.push_back("\t- " + error);
  }
  return message;
}

RegisterParser<AutowareValidatorOsmParser> regParser;
}  // namespace

std::unique_ptr<LaneletMap> AutowareValidatorOsmParser::parse(
  const std::string & filename, ErrorMessages & errors) const
{
  // read xml
  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError(
      std::string("Errors occured while parsing osm file: ") + result.description());
  }
  osm::Errors osmReadErrors;
  testAndPrintLocaleWarning(osmReadErrors);
  auto file = lanelet::osm::autoware::read(doc, &osmReadErrors);
  auto map = fromOsmFile(file, errors);
  // make sure ids in the file are known to Lanelet2 id management.
  registerIds(file.nodes);
  registerIds(file.ways);
  registerIds(file.relations);
  errors = buildErrorMessage(
    "Errors ocurred while parsing Lanelet Map:", utils::concatenate({osmReadErrors, errors}));
  return map;
}
}  // namespace lanelet::io_handlers

namespace lanelet::osm
{
namespace keyword
{
constexpr const char * Osm = "osm";
constexpr const char * Tag = "tag";
constexpr const char * Key = "k";
constexpr const char * Value = "v";
constexpr const char * Node = "node";
constexpr const char * Way = "way";
constexpr const char * Relation = "relation";
constexpr const char * Member = "member";
constexpr const char * Role = "role";
constexpr const char * Type = "type";
constexpr const char * Nd = "nd";
constexpr const char * Ref = "ref";
constexpr const char * Id = "id";
constexpr const char * Lat = "lat";
constexpr const char * Lon = "lon";
constexpr const char * Version = "version";
constexpr const char * Visible = "visible";
constexpr const char * Elevation = "ele";
constexpr const char * Action = "action";
constexpr const char * Delete = "delete";
}  // namespace keyword

struct UnresolvedRole
{
  Id relation{};
  Id referencedRelation{};
  Primitive ** location{};
};

Attributes tags(const pugi::xml_node & node)
{
  Attributes attributes;
  for (auto tag = node.child(keyword::Tag); tag;  // NOLINT
       tag = tag.next_sibling(keyword::Tag)) {
    if (std::string(tag.attribute(keyword::Key).value()) == keyword::Elevation) {
      continue;
    }
    attributes[tag.attribute(keyword::Key).value()] = tag.attribute(keyword::Value).value();
  }
  return attributes;
}

bool isDeleted(const pugi::xml_node & node)
{
  auto action = node.attribute(keyword::Action);
  return action && std::string(action.value()) == keyword::Delete;  // NOLINT
}

void removeAndFixPlaceholders(
  Primitive ** toRemove, Roles & fromRoles, std::vector<UnresolvedRole> & placeholders)
{
  // find other placeholders that we have to fix
  auto remIt = std::find_if(fromRoles.begin(), fromRoles.end(), [&](const Role & role) {
    return &role.second == toRemove;
  });
  assert(remIt != fromRoles.end());
  std::vector<std::pair<size_t, Primitive **>> placeholderLocations;
  for (auto it = fromRoles.begin(); it != fromRoles.end(); ++it) {
    if (it->second == nullptr && remIt != it) {
      auto idx = std::distance(fromRoles.begin(), it);
      placeholderLocations.emplace_back(it > remIt ? idx - 1 : idx, &it->second);
    }
  }
  fromRoles.erase(remIt);
  if (placeholderLocations.empty()) {
    return;  // nothing to update
  }
  // get the new locations
  std::map<Primitive **, Primitive **> newLocations;
  for (auto & loc : placeholderLocations) {
    newLocations.emplace(
      loc.second, &std::next(fromRoles.begin(), static_cast<int64_t>(loc.first))->second);
  }
  // adapt existing locations
  for (auto & placeholder : placeholders) {
    auto it = newLocations.find(placeholder.location);
    if (it != newLocations.end()) {
      placeholder.location = it->second;
    }
  }
}

namespace autoware
{

class AutowareValidatorOsmParser
{
public:
  static File read(const pugi::xml_node & fileNode, Errors * errors = nullptr)
  {
    AutowareValidatorOsmParser autowareValidatorOsmParser;
    File file;
    auto osmNode = fileNode.child(keyword::Osm);
    file.nodes = autowareValidatorOsmParser.readNodes(osmNode);
    file.ways = autowareValidatorOsmParser.readWays(osmNode, file.nodes);
    file.relations = autowareValidatorOsmParser.readRelations(osmNode, file.nodes, file.ways);
    if (errors != nullptr) {
      *errors = autowareValidatorOsmParser.errors_;
    }
    return file;
  }

private:
  Nodes readNodes(const pugi::xml_node & osmNode)
  {
    Nodes nodes;
    for (auto node = osmNode.child(keyword::Node); node;  // NOLINT
         node = node.next_sibling(keyword::Node)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto lat = node.attribute(keyword::Lat).as_double(0.);
      const auto lon = node.attribute(keyword::Lon).as_double(0.);
      const auto elevationNode =
        node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation);
      if (!elevationNode) {
        reportParseError(id, "Elevation tag is not defined for the given node.");
      } else if (!elevationNode.attribute(keyword::Value)) {
        reportParseError(id, "Elevation tag exists but does not have a value.");
      }
      const auto ele = elevationNode.attribute(keyword::Value).as_double(.0);

      nodes[id] = Node{id, attributes, {lat, lon, ele}};
    }
    return nodes;
  }

  Ways readWays(const pugi::xml_node & osmNode, Nodes & nodes)
  {
    Ways ways;
    for (auto node = osmNode.child(keyword::Way); node;  // NOLINT
         node = node.next_sibling(keyword::Way)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto nodeIds = [&node] {
        Ids ids;
        for (auto refNode = node.child(keyword::Nd); refNode;  // NOLINT
             refNode = refNode.next_sibling(keyword::Nd)) {
          ids.push_back(refNode.attribute(keyword::Ref).as_llong());
        }
        return ids;
      }();
      std::vector<Node *> wayNodes;
      try {
        wayNodes =
          utils::transform(nodeIds, [&nodes](const auto & elem) { return &nodes.at(elem); });
      } catch (std::out_of_range &) {
        reportParseError(id, "Way references nonexisting points");
      }
      ways[id] = Way{id, attributes, wayNodes};
    }
    return ways;
  }

  Relations readRelations(const pugi::xml_node & osmNode, Nodes & nodes, Ways & ways)
  {
    Relations relations;
    // Two-pass approach: We can resolve all roles except where relations reference relations. We
    // insert a dummy nullptr and resolve that later on.
    std::vector<UnresolvedRole> unresolvedRoles;
    for (auto node = osmNode.child(keyword::Relation); node;  // NOLINT
         node = node.next_sibling(keyword::Relation)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      auto & relation = relations.emplace(id, Relation{id, attributes, {}}).first->second;

      // resolve members
      auto & roles = relation.members;
      for (auto member = node.child(keyword::Member); member;  // NOLINT
           member = member.next_sibling(keyword::Member)) {
        Id memberId = member.attribute(keyword::Ref).as_llong();
        const std::string role = member.attribute(keyword::Role).value();
        const std::string type = member.attribute(keyword::Type).value();
        try {
          if (type == keyword::Node) {
            roles.emplace_back(role, &nodes.at(memberId));
          } else if (type == keyword::Way) {
            roles.emplace_back(role, &ways.at(memberId));
          } else if (type == keyword::Relation) {
            // insert a placeholder and store a pointer to it for the second pass
            roles.emplace_back(role, nullptr);
            unresolvedRoles.push_back(UnresolvedRole{id, memberId, &roles.back().second});
          }
        } catch (std::out_of_range &) {
          reportParseError(id, "Relation has nonexistent member " + std::to_string(memberId));
        }
      }
    }

    // now resolve all unresolved roles that point to other relations
    for (const auto & unresolvedRole : unresolvedRoles) {
      try {
        assert(*unresolvedRole.location == nullptr);
        *unresolvedRole.location = &relations.at(unresolvedRole.referencedRelation);
      } catch (std::out_of_range &) {
        reportParseError(
          unresolvedRole.relation, "Relation references nonexistent relation " +
                                     std::to_string(unresolvedRole.referencedRelation));
        // now it gets ugly: find placeholder and remove it. Fix all other placeholders because the
        // pointers are invalidated after moving. This is inefficent, but its the fault of the guy
        // loading an invalid map, not ours.
        auto & relation = relations.at(unresolvedRole.relation);
        removeAndFixPlaceholders(unresolvedRole.location, relation.members, unresolvedRoles);
      }
    }
    return relations;
  }

  AutowareValidatorOsmParser() = default;
  void reportParseError(Id id, const std::string & what)
  {
    auto errstr = "Error reading primitive with id " + std::to_string(id) + " from file: " + what;
    errors_.push_back(errstr);
  }
  Errors errors_;
};

File read(pugi::xml_document & node, Errors * errors)
{
  return AutowareValidatorOsmParser::read(node, errors);
}
}  // namespace autoware
}  // namespace lanelet::osm
