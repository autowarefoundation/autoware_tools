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

#ifndef LANELET2_MAP_VALIDATOR__UTILS_HPP_
#define LANELET2_MAP_VALIDATOR__UTILS_HPP_

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

#include <string>
#include <type_traits>
#include <vector>

namespace lanelet::autoware::validation
{
template <typename Container, typename T>
auto appendIssues(std::vector<T> & to, Container && from) ->
  typename std::enable_if_t<std::is_same_v<T, typename std::decay_t<Container>::value_type>, void>
{
  if constexpr (std::is_rvalue_reference<decltype(from)>::value) {
    to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
  }
  if constexpr (std::is_lvalue_reference<decltype(from)>::value) {
    to.insert(to.end(), from.begin(), from.end());
  }
}

template <typename T>
void checkPrimitivesType(
  std::vector<T> & in_vec, const std::string & expected_type,
  const lanelet::validation::Issue & issue, lanelet::validation::Issues & issues)
{
  for (auto iter = in_vec.begin(); iter != in_vec.end(); ++iter) {
    const auto & item = *iter;
    const auto & attrs = item.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Type);
    if (it == attrs.end() || it->second != expected_type) {
      issues.emplace_back(issue.severity, issue.primitive, item.id(), issue.message);
      const auto new_it = in_vec.erase(iter);
      if (new_it == in_vec.end()) {
        break;
      }
      iter = new_it;
    }
  }
}

template <typename T>
void checkPrimitivesType(
  std::vector<T> & in_vec, const std::string & expected_type, const std::string & expected_subtype,
  const lanelet::validation::Issue & issue, lanelet::validation::Issues & issues)
{
  for (auto iter = in_vec.begin(); iter != in_vec.end(); ++iter) {
    const auto & item = *iter;
    const auto & attrs = item.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Type);
    const auto & it_sub = attrs.find(lanelet::AttributeName::Subtype);
    if (
      it == attrs.end() || it->second != expected_type || it_sub == attrs.end() ||
      it_sub->second != expected_subtype) {
      issues.emplace_back(issue.severity, issue.primitive, item.id(), issue.message);
      const auto new_it = in_vec.erase(iter);
      if (new_it == in_vec.end()) {
        break;
      }
      iter = new_it;
    }
  }
}

}  // namespace lanelet::autoware::validation

std::string snake_to_upper_camel(const std::string & snake_case);
std::string issue_code(const std::string & name, const int number);
std::string issue_code_prefix(const std::string & name, const int number);

#endif  // LANELET2_MAP_VALIDATOR__UTILS_HPP_
