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

#include "perception_replayer.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

using autoware::planning_debug_tools::PerceptionReplayer;
using autoware::planning_debug_tools::PerceptionReplayerCommonParam;

int main(int argc, char * argv[])
{
  try {
    // Create arguments for Qt application
    std::vector<std::string> non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char *> qt_argv;
    qt_argv.push_back(argv[0]);
    for (auto & arg : non_ros_args) {
      qt_argv.push_back(const_cast<char *>(arg.c_str()));
    }
    int qt_argc = static_cast<int>(qt_argv.size());

    QApplication app(qt_argc, qt_argv.data());
    app.setApplicationName("perception_replayer");
    app.setApplicationVersion(PACKAGE_VERSION);

    QCommandLineParser parser;
    parser.setApplicationDescription(
      "This script can overlay the perception results from the rosbag on the planning simulator.\n"
      "\n"
      "In detail, this script publishes the data at a certain timestamp from the rosbag.\n"
      "The timestamp will increase according to the real time without any operation.\n"
      "By using the GUI, you can modify the timestamp by pausing, changing the rate or going back "
      "into the past.");

    QCommandLineOption bag_option(
      QStringList() << "b"
                    << "bag",
      "rosbag", "BAG");
    parser.addOption(bag_option);

    QCommandLineOption detected_object_option(
      "detected-object", "Use detected objects (default: false)");
    parser.addOption(detected_object_option);

    QCommandLineOption tracked_object_option(
      "tracked-object", "Use tracked objects (default: false)");
    parser.addOption(tracked_object_option);

    QCommandLineOption rosbag_format_option(
      QStringList() << "f"
                    << "rosbag-format",
      "rosbag data format (default is sqlite3)", "{sqlite3,mcap}", "sqlite3");
    parser.addOption(rosbag_format_option);

    QCommandLineOption help_option(
      QStringList() << "h"
                    << "help",
      "show this help message and exit");
    parser.addOption(help_option);

    parser.addVersionOption();

    parser.process(app);

    const auto show_help = [&]() {
      std::cout << "usage: ros2 run planning_debug_tools perception_replayer [-h] [-b BAG] [-d] "
                   "[-t] [-f {sqlite3,mcap}] [-u TXT] [-v]"
                << std::endl
                << std::endl;
      std::cout << "options:" << std::endl;
      std::cout << "    -h, --help            show this help message and exit" << std::endl;
      std::cout << "    -b BAG, --bag BAG     rosbag" << std::endl;
      std::cout << "    -d, --detected-object" << std::endl;
      std::cout << "                          publish detected object" << std::endl;
      std::cout << "    -t, --tracked-object  publish tracked object" << std::endl;
      std::cout << "    -f {sqlite3,mcap}, --rosbag-format {sqlite3,mcap}" << std::endl;
      std::cout << "                          rosbag data format (default is sqlite3)" << std::endl;
      std::cout << "    -v, --version         displays version information" << std::endl;
    };

    // Check if help was requested
    if (parser.isSet(help_option)) {
      show_help();
      return 0;
    }

    PerceptionReplayerCommonParam perception_replayer_param;
    perception_replayer_param.rosbag_path = parser.value(bag_option).toStdString();

    // If rosbag_path is empty, print error and exit
    if (perception_replayer_param.rosbag_path.empty()) {
      show_help();
      return 1;
    }

    perception_replayer_param.rosbag_format = parser.value(rosbag_format_option).toStdString();
    if (
      perception_replayer_param.rosbag_format != "sqlite3" &&
      perception_replayer_param.rosbag_format != "mcap") {
      std::cerr << "Error: invalid rosbag format: " << perception_replayer_param.rosbag_format
                << std::endl;
      return 1;
    }

    perception_replayer_param.detected_object = parser.isSet(detected_object_option);
    perception_replayer_param.tracked_object = parser.isSet(tracked_object_option);

    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<PerceptionReplayer>(perception_replayer_param, node_options);

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      app.processEvents();
    }
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::cerr << "Exception in main(): " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return 1;
  }
  return 0;
}
