// Copyright 2025 TIER IV, Inc.
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

#ifndef PERCEPTION_REPLAYER__HELP_UTILS_HPP_
#define PERCEPTION_REPLAYER__HELP_UTILS_HPP_

#include "utils.hpp"

#include <QCommandLineParser>
#include <QCoreApplication>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

void show_help(
  const QList<QCommandLineOption> & options, const std::string & execute_command,
  const std::string & description)
{
  // construct usage options
  std::string options_str = {};
  std::vector<std::pair<std::string, std::string>> option_names_and_values;
  option_names_and_values.reserve(options.size());

  for (const auto & option : options) {
    std::string names;
    const std::string value = option.valueName().toStdString();
    for (const auto & name : option.names()) {
      const std::string unit_option =
        (name.size() == 1 ? "-" : "--") + name.toStdString() + (value.empty() ? "" : " " + value);
      if (!names.empty()) {
        names += ", ";
      } else {
        options_str += " [" + unit_option + "]";
      }

      names += unit_option;
    }

    const std::string default_value =
      option.defaultValues().isEmpty()
        ? ""
        : " (default: " + option.defaultValues().join(", ").toStdString() + ")";

    option_names_and_values.emplace_back(names, option.description().toStdString() + default_value);
  }

  std::cout << "usage: " << execute_command << options_str << std::endl << std::endl;

  std::cout << "version: " << QCoreApplication::applicationVersion().toStdString() << std::endl
            << std::endl;

  std::cout << description << std::endl << std::endl;

  std::cout << "options:" << std::endl;

  constexpr size_t DESCRIPTION_INDENT = 26;
  for (const auto & option : option_names_and_values) {
    const std::string key_str = std::string(2, ' ') + option.first;
    std::cout << key_str;
    if (key_str.length() < DESCRIPTION_INDENT) {
      std::cout << std::string(DESCRIPTION_INDENT - key_str.length(), ' ');
    } else {
      std::cout << std::endl << std::string(DESCRIPTION_INDENT, ' ');
    }
    std::cout << option.second << std::endl;
  }
}

QList<QCommandLineOption> make_object_filter_options()
{
  return {
    QCommandLineOption(
      QStringList() << "filter-radius",
      "publish only objects within this 2D distance range [m] from the recorded ego pose "
      "(<max> or <min>,<max>)",
      "radius"),
    QCommandLineOption(
      QStringList() << "filter-angle",
      "publish only objects within this angle range [deg] in the recorded ego frame "
      "(<min>,<max>, 0 is front and counterclockwise is positive, wraps through the rear if "
      "min > max)",
      "angle"),
    QCommandLineOption(
      QStringList() << "filter-class",
      "publish only objects of these comma-separated classes (e.g., 'car,pedestrian')", "classes"),
    QCommandLineOption(
      QStringList() << "filter-invert",
      "remove the objects matching the filter options instead of keeping them")};
}

QStringList split_numbers(const QString & value)
{
  const QStringList values = value.split(',', Qt::SkipEmptyParts);
  const auto is_number = [](const QString & str) {
    bool ok = false;
    str.toDouble(&ok);
    return ok;
  };
  return std::all_of(values.begin(), values.end(), is_number) ? values : QStringList{};
}

bool parse_filter_radius(
  const QString & value, autoware::planning_debug_tools::utils::ObjectFilterParam & filter)
{
  const QStringList values = split_numbers(value);
  if (values.size() == 1) {
    filter.max_radius = values[0].toDouble();
  } else if (values.size() == 2) {
    filter.min_radius = values[0].toDouble();
    filter.max_radius = values[1].toDouble();
  }
  if (
    values.isEmpty() || values.size() > 2 || filter.min_radius < 0.0 ||
    filter.max_radius <= filter.min_radius) {
    std::cerr << "Error: --filter-radius must be <max> or <min>,<max> with 0 <= min < max"
              << std::endl;
    return false;
  }
  return true;
}

bool parse_filter_angle(
  const QString & value, autoware::planning_debug_tools::utils::ObjectFilterParam & filter)
{
  const QStringList values = split_numbers(value);
  const auto is_valid = [](const double deg) { return -180.0 <= deg && deg <= 180.0; };
  if (values.size() != 2 || !is_valid(values[0].toDouble()) || !is_valid(values[1].toDouble())) {
    std::cerr << "Error: --filter-angle must be <min>,<max> within [-180, 180]" << std::endl;
    return false;
  }
  filter.min_angle = values[0].toDouble() * M_PI / 180.0;
  filter.max_angle = values[1].toDouble() * M_PI / 180.0;
  return true;
}

bool parse_filter_class(
  const QString & value, autoware::planning_debug_tools::utils::ObjectFilterParam & filter)
{
  using autoware::planning_debug_tools::ObjectClassification;
  const std::unordered_map<std::string, uint8_t> class_map = {
    {"UNKNOWN", ObjectClassification::UNKNOWN}, {"CAR", ObjectClassification::CAR},
    {"TRUCK", ObjectClassification::TRUCK},     {"BUS", ObjectClassification::BUS},
    {"TRAILER", ObjectClassification::TRAILER}, {"MOTORCYCLE", ObjectClassification::MOTORCYCLE},
    {"BICYCLE", ObjectClassification::BICYCLE}, {"PEDESTRIAN", ObjectClassification::PEDESTRIAN}};
  for (const auto & name : value.split(',', Qt::SkipEmptyParts)) {
    const auto it = class_map.find(name.trimmed().toUpper().toStdString());
    if (it == class_map.end()) {
      std::cerr << "Error: unknown class for --filter-class: " << name.toStdString() << std::endl;
      return false;
    }
    filter.classes.push_back(it->second);
  }
  if (filter.classes.empty()) {
    std::cerr << "Error: --filter-class must be <class,...> with at least one class" << std::endl;
    return false;
  }
  return true;
}

bool parse_object_filter_options(
  const QCommandLineParser & parser,
  autoware::planning_debug_tools::utils::ObjectFilterParam & filter)
{
  filter.enabled =
    parser.isSet("filter-radius") || parser.isSet("filter-angle") || parser.isSet("filter-class");
  filter.invert = parser.isSet("filter-invert");
  if (filter.invert && !filter.enabled) {
    std::cerr << "Error: --filter-invert requires at least one filter option" << std::endl;
    return false;
  }
  return (!parser.isSet("filter-radius") ||
          parse_filter_radius(parser.value("filter-radius"), filter)) &&
         (!parser.isSet("filter-angle") ||
          parse_filter_angle(parser.value("filter-angle"), filter)) &&
         (!parser.isSet("filter-class") ||
          parse_filter_class(parser.value("filter-class"), filter));
}

#endif  // PERCEPTION_REPLAYER__HELP_UTILS_HPP_
