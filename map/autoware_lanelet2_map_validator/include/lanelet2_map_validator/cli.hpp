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

#ifndef LANELET2_MAP_VALIDATOR__CLI_HPP_
#define LANELET2_MAP_VALIDATOR__CLI_HPP_

#include <boost/program_options.hpp>

#include <lanelet2_validation/Cli.h>

#include <iostream>
#include <string>

namespace lanelet
{
namespace autoware
{
namespace validation
{
struct MetaConfig
{
  lanelet::validation::CommandLineConfig command_line_config;
  std::string projector_type;
  std::string requirements_file;
  std::string output_file_path;
};

MetaConfig parseCommandLine(int argc, const char * argv[]);

}  // namespace validation
}  // namespace autoware
}  // namespace lanelet

#endif  // LANELET2_MAP_VALIDATOR__CLI_HPP_
