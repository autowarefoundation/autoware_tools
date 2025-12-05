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

#include "perception_reproducer.hpp"

#include <QCommandLineParser>
#include <QCoreApplication>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  try {
    std::vector<std::string> non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    std::vector<char *> qt_argv;
    qt_argv.push_back(argv[0]);
    for (auto & arg : non_ros_args) {
      qt_argv.push_back(const_cast<char *>(arg.c_str()));
    }
    int qt_argc = static_cast<int>(qt_argv.size());

    QCoreApplication app(qt_argc, qt_argv.data());
    app.setApplicationName("perception_reproducer");
    app.setApplicationVersion(PACKAGE_VERSION);

    QCommandLineParser parser;
    parser.setApplicationDescription(
      "Replay perception outputs from rosbag data to reproduce planning behaviour.");

    const QCommandLineOption bag_option(
      QStringList() << "b"
                    << "bag",
      "rosbag", "BAG");
    parser.addOption(bag_option);

    const QCommandLineOption detected_object_option(
      "detected-object", "Use detected objects (default: false)");
    parser.addOption(detected_object_option);

    const QCommandLineOption tracked_object_option(
      "tracked-object", "Use tracked objects (default: false)");
    parser.addOption(tracked_object_option);

    const QCommandLineOption rosbag_format_option(
      QStringList() << "f"
                    << "rosbag-format",
      "rosbag data format (default is sqlite3)", "{sqlite3,mcap}", "sqlite3");
    parser.addOption(rosbag_format_option);

    const QCommandLineOption search_radius_option(
      QStringList() << "r"
                    << "search-radius",
      "Search radius for matching ego odom (meters).", "radius", "1.5");
    parser.addOption(search_radius_option);

    const QCommandLineOption cool_down_option(
      QStringList() << "c"
                    << "cool-down",
      "Cool-down time before reproducing the same timestamp (seconds).", "seconds", "80.0");
    parser.addOption(cool_down_option);

    const QCommandLineOption noise_option(
      QStringList() << "n"
                    << "noise",
      "Apply perception noise to objects (default: true)");
    parser.addOption(noise_option);

    const QCommandLineOption verbose_option(
      QStringList() << "v"
                    << "verbose",
      "Output debug data.");
    parser.addOption(verbose_option);

    const QCommandLineOption help_option(
      QStringList() << "h"
                    << "help",
      "show this help message and exit");
    parser.addOption(help_option);

    parser.process(app);

    const auto show_help = [&]() {
      std::cout << "usage: ros2 run planning_debug_tools perception_reproducer [-h] [-b BAG] [-d] "
                   "[-t] [-f {sqlite3,mcap}] [-u TXT] [-n] [-r <radius>] [-c <seconds>] [-v]"
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
      std::cout << "    -n, --noise           apply perception noise to objects (default: true)"
                << std::endl;
      std::cout << "    -r <radius>, --search-radius <radius>" << std::endl;
      std::cout << "                          search radius for matching ego odom (default: 1.5 m)"
                << std::endl;
      std::cout << "    -c <seconds>, --cool-down <seconds>" << std::endl;
      std::cout
        << "                          cool-down time before reusing timestamp (default: 80.0 s)"
        << std::endl;
      std::cout << "    -v, --verbose         output debug data" << std::endl;
    };

    if (parser.isSet(help_option)) {
      show_help();
      return 0;
    }

    if (!parser.isSet(bag_option)) {
      std::cerr << "Error: bag path is required." << std::endl << std::endl;
      show_help();
      return 1;
    }

    autoware::planning_debug_tools::PerceptionReproducerParam param;
    param.rosbag_path = parser.value(bag_option).toStdString();
    param.rosbag_format = parser.value(rosbag_format_option).toStdString();
    param.detected_object = parser.isSet(detected_object_option);
    param.tracked_object = parser.isSet(tracked_object_option);
    param.search_radius = 1.5;
    param.reproduce_cool_down = 80.0;
    param.noise = true;
    param.verbose = parser.isSet(verbose_option);

    if (param.rosbag_format != "sqlite3" && param.rosbag_format != "mcap") {
      std::cerr << "Error: invalid rosbag format: " << param.rosbag_format << std::endl;
      return 1;
    }

    bool ok = false;
    const auto search_radius_str = parser.value(search_radius_option);
    const double search_radius = search_radius_str.toDouble(&ok);
    if (!ok) {
      std::cerr << "Error: invalid search radius: " << search_radius_str.toStdString() << std::endl;
      return 1;
    }
    param.search_radius = search_radius;

    const auto cool_down_str = parser.value(cool_down_option);
    const double cool_down = cool_down_str.toDouble(&ok);
    if (!ok) {
      std::cerr << "Error: invalid cool-down value: " << cool_down_str.toStdString() << std::endl;
      return 1;
    }
    param.reproduce_cool_down = cool_down;

    if (param.search_radius == 0.0) {
      param.reproduce_cool_down = 0.0;
    }

    auto node = std::make_shared<autoware::planning_debug_tools::PerceptionReproducer>(
      param, rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::cerr << "Exception in main(): " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  return 0;
}
