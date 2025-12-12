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

#include <QCommandLineParser>
#include <QCoreApplication>

#include <iostream>
#include <memory>
#include <string>
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

#endif  // PERCEPTION_REPLAYER__HELP_UTILS_HPP_
