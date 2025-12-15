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

#include "autoware_planning_data_analyzer_node.hpp"

// #include "closed_loop_evaluator.hpp"  // TODO(gosakayori): Closed loop evaluator not yet
// implemented
#include "open_loop_evaluator.hpp"
#include "or_scene_evaluator.hpp"

#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <autoware_map_msgs/msg/detail/lanelet_map_bin__struct.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <rmw/rmw.h>

#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{
using autoware_utils::create_marker_color;
using autoware_utils_rclcpp::get_or_declare_parameter;

// Helper function to get parameter with default value
template <typename T>
T get_parameter_or_default(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_value);
}

AutowarePlanningDataAnalyzerNode::AutowarePlanningDataAnalyzerNode(
  const rclcpp::NodeOptions & node_options)
: Node("autoware_planning_data_analyzer", node_options),
  route_handler_{std::make_shared<autoware::route_handler::RouteHandler>()}
{
  setup_evaluation_bag_writer();

  // Open bag file
  bag_path_ = get_or_declare_parameter<std::string>(*this, "bag_path");
  try {
    bag_reader_.open(bag_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open bag file: %s", e.what());
    throw;
  }

  // Initialize topic names from parameters with defaults
  map_topic_name_ = get_parameter_or_default<std::string>(*this, "map_topic", "/map/vector_map");
  map_marker_topic_name_ =
    get_parameter_or_default<std::string>(*this, "map_marker_array", "/map/vector_map_marker");
  route_topic_name_ =
    get_parameter_or_default<std::string>(*this, "route_topic", "/planning/mission_planning/route");
  odometry_topic_name_ =
    get_parameter_or_default<std::string>(*this, "odometry_topic", "/localization/kinematic_state");
  trajectory_topic_name_ = get_parameter_or_default<std::string>(
    *this, "trajectory_topic", "/planning/scenario_planning/lane_driving/trajectory");
  objects_topic_name_ = get_parameter_or_default<std::string>(
    *this, "objects_topic", "/perception/object_recognition/objects");
  tf_topic_name_ = get_parameter_or_default<std::string>(*this, "tf_topic", "/tf");
  acceleration_topic_name_ = get_parameter_or_default<std::string>(
    *this, "acceleration_topic", "/localization/acceleration");
  steering_topic_name_ = get_parameter_or_default<std::string>(
    *this, "steering_topic", "/vehicle/status/steering_status");

  // Read evaluation mode
  const auto mode_str =
    get_parameter_or_default<std::string>(*this, "evaluation.mode", "closed_loop");
  if (mode_str == "open_loop") {
    evaluation_mode_ = EvaluationMode::OPEN_LOOP;
  } else if (mode_str == "or_scene") {
    evaluation_mode_ = EvaluationMode::OR_SCENE;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid evaluation mode: %s. Using OPEN_LOOP.", mode_str.c_str());
    evaluation_mode_ = EvaluationMode::OPEN_LOOP;
  }

  run_evaluation();
}

AutowarePlanningDataAnalyzerNode::~AutowarePlanningDataAnalyzerNode()
{
  if (evaluation_bag_writer_) {
    try {
      evaluation_bag_writer_->close();
      RCLCPP_INFO(get_logger(), "Evaluation bag writer closed successfully");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error closing evaluation bag writer: %s", e.what());
    }
  }
}

void AutowarePlanningDataAnalyzerNode::setup_evaluation_bag_writer()
{
  try {
    evaluation_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Get output bag path from parameters with default
    auto output_bag_path =
      get_parameter_or_default<std::string>(*this, "evaluation_output_bag_path", "./");

    // Ensure directory exists
    const auto output_dir = std::filesystem::path(output_bag_path).parent_path();
    RCLCPP_INFO(get_logger(), "Output directory: %s", output_dir.string().c_str());
    if (!output_dir.empty() && !std::filesystem::exists(output_dir)) {
      std::filesystem::create_directories(output_dir);
      RCLCPP_INFO(get_logger(), "Created output directory: %s", output_dir.string().c_str());
    }

    // Validate write permissions
    if (!output_dir.empty()) {
      const auto test_file = output_dir / ".write_test";
      std::ofstream test_stream(test_file);
      if (!test_stream.is_open()) {
        throw std::runtime_error("No write permission to output directory: " + output_dir.string());
      }
      test_stream.close();
      std::filesystem::remove(test_file);
    }

    // Setup bag writer with MCAP format
    const rosbag2_storage::StorageOptions storage_options{output_bag_path, "mcap"};

    const rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    evaluation_bag_writer_->open(storage_options, converter_options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to setup evaluation bag writer: %s", e.what());
    evaluation_bag_writer_ = nullptr;
  }
}

void AutowarePlanningDataAnalyzerNode::run_evaluation()
{
  RCLCPP_INFO(get_logger(), "Starting evaluation...");

  // Storage for the last route message and tf messages
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr last_route_msg = nullptr;
  tf2_msgs::msg::TFMessage tf_static_msg;
  std::vector<std::pair<tf2_msgs::msg::TFMessage, rclcpp::Time>> tf_messages;

  // Quick scan for route and tf messages
  while (bag_reader_.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader_.read_next();
    const auto & topic_name = serialized_message->topic_name;

    if (topic_name == map_topic_name_) {
      try {
        autoware_map_msgs::msg::LaneletMapBin msg;
        rclcpp::Serialization<autoware_map_msgs::msg::LaneletMapBin> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        route_handler_->setMap(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize map message: %s", e.what());
      }
    } else if (topic_name == map_marker_topic_name_) {
      try {
        visualization_msgs::msg::MarkerArray msg;
        rclcpp::Serialization<visualization_msgs::msg::MarkerArray> serializer;
        rclcpp::SerializedMessage serialize_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialize_msg, &msg);
        map_marker_ = std::make_shared<visualization_msgs::msg::MarkerArray>(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize map marker message: %s", e.what());
      }
    } else if (topic_name == route_topic_name_) {
      try {
        autoware_planning_msgs::msg::LaneletRoute msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::LaneletRoute> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        last_route_msg = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize route message: %s", e.what());
      }
    } else if (topic_name == "/tf_static") {
      try {
        tf2_msgs::msg::TFMessage msg;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        // Accumulate all tf_static transforms
        tf_static_msg.transforms.insert(
          tf_static_msg.transforms.end(), msg.transforms.begin(), msg.transforms.end());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize tf_static message: %s", e.what());
      }
    }
  }

  // Route is optional for OR scene evaluation (doesn't use lane-based metrics)
  if (!last_route_msg && evaluation_mode_ != EvaluationMode::OR_SCENE) {
    RCLCPP_WARN(get_logger(), "No route message found in bag. Evaluation aborted.");
    return;
  }

  if (last_route_msg) {
    route_handler_->setRoute(*last_route_msg);
  } else {
    RCLCPP_INFO(get_logger(), "No route found in bag (OK for OR scene evaluation)");
  }

  // Seek back to the beginning of the bag for mode-specific evaluation
  bag_reader_.seek(0);

  // Store reference times for map/route markers
  rclcpp::Time start_time = now();
  rclcpp::Time end_time = now();

  // Create topic names structure
  TopicNames topic_names;
  topic_names.route_topic = route_topic_name_;
  topic_names.odometry_topic = odometry_topic_name_;
  topic_names.trajectory_topic = trajectory_topic_name_;
  topic_names.objects_topic = objects_topic_name_;
  topic_names.tf_topic = tf_topic_name_;
  topic_names.acceleration_topic = acceleration_topic_name_;
  topic_names.steering_topic = steering_topic_name_;

  switch (evaluation_mode_) {
    case EvaluationMode::OPEN_LOOP: {
      OpenLoopEvaluator evaluator(get_logger(), route_handler_);
      auto times =
        evaluator.run_evaluation_from_bag(bag_path_, evaluation_bag_writer_.get(), topic_names);
      start_time = times.first;
      end_time = times.second;
      break;
    }
    case EvaluationMode::OR_SCENE: {
      // Read OR scene specific parameters
      const double time_window_sec =
        get_parameter_or_default<double>(*this, "or_scene_evaluation.time_window_sec", 0.5);
      const bool enable_debug_viz = get_parameter_or_default<bool>(
        *this, "or_scene_evaluation.enable_debug_visualization", false);

      // Read success criteria
      ORSuccessCriteria success_criteria;
      success_criteria.enabled = get_parameter_or_default<bool>(
        *this, "or_scene_evaluation.success_criteria.enabled", false);
      success_criteria.max_ade = get_parameter_or_default<double>(
        *this, "or_scene_evaluation.success_criteria.max_ade", 1.0);
      success_criteria.max_fde = get_parameter_or_default<double>(
        *this, "or_scene_evaluation.success_criteria.max_fde", 1.5);
      success_criteria.max_lateral_deviation = get_parameter_or_default<double>(
        *this, "or_scene_evaluation.success_criteria.max_lateral_deviation", 0.5);
      success_criteria.min_ttc = get_parameter_or_default<double>(
        *this, "or_scene_evaluation.success_criteria.min_ttc", 3.0);

      const auto debug_output_dir = get_parameter_or_default<std::string>(
        *this, "or_scene_evaluation.debug_output_dir", "~/or_scene_debug_images");

      ORSceneEvaluator evaluator(
        get_logger(), route_handler_, time_window_sec, success_criteria, enable_debug_viz,
        debug_output_dir);

      // Set OR events JSON paths if provided
      const auto or_events_input_path = get_parameter_or_default<std::string>(
        *this, "or_scene_evaluation.or_events_input_path", "");
      const auto or_events_output_path = get_parameter_or_default<std::string>(
        *this, "or_scene_evaluation.or_events_output_path", "");
      const auto input_bag_path =
        get_parameter_or_default<std::string>(*this, "or_scene_evaluation.input_bag_path", "");

      if (!or_events_input_path.empty()) {
        evaluator.set_or_events_json_path(or_events_input_path);
      }
      if (!or_events_output_path.empty()) {
        evaluator.set_or_events_output_path(or_events_output_path);
      }
      if (!input_bag_path.empty()) {
        evaluator.set_input_bag_path(input_bag_path);
      }

      // Set map path for visualization
      auto map_path = get_or_declare_parameter<std::string>(*this, "or_scene_evaluation.map_path");
      if (!map_path.empty()) {
        evaluator.set_map_path(map_path);
      }

      // Extract metric topic prefix from trajectory topic for multi-run support
      // Topics like /model_v1/planning/.../trajectory → prefix "model_v1"
      // Standard topics like /planning/.../trajectory → no prefix
      std::string trajectory_topic = topic_names.trajectory_topic;
      if (!trajectory_topic.empty() && trajectory_topic[0] == '/') {
        // Find second slash (end of first component)
        size_t second_slash = trajectory_topic.find('/', 1);
        if (second_slash != std::string::npos) {
          std::string first_component = trajectory_topic.substr(1, second_slash - 1);

          // Check if first component is NOT a standard Autoware namespace
          const std::set<std::string> standard_namespaces = {
            "planning", "control", "localization", "perception",
            "sensing",  "map",     "system",       "vehicle"};

          if (standard_namespaces.find(first_component) == standard_namespaces.end()) {
            // This is a custom prefix for multi-run collection
            evaluator.set_metric_topic_prefix(first_component);
            RCLCPP_INFO(get_logger(), "Using metric topic prefix: %s", first_component.c_str());
          }
        }
      }

      auto times =
        evaluator.run_evaluation_from_bag(bag_path_, evaluation_bag_writer_.get(), topic_names);
      start_time = times.first;
      end_time = times.second;
      break;
    }
  }

  // Write tf_static at the beginning if available
  if (
    !tf_static_msg.transforms.empty() && evaluation_bag_writer_ && start_time.seconds() > 0 &&
    end_time.seconds() > 0) {
    // Use start_time so timestamps align with rest of bag (not 0 which breaks Lichtblick)
    rclcpp::Time tf_time = start_time;

    // Set timestamps in the transforms to start_time
    tf2_msgs::msg::TFMessage timestamped_tf_static = tf_static_msg;
    for (auto & transform : timestamped_tf_static.transforms) {
      transform.header.stamp = tf_time;
    }

    evaluation_bag_writer_->write(timestamped_tf_static, "/tf_static", tf_time);
  }

  // Write map and route markers with start_time (not 0)
  write_map_and_route_markers_to_bag(start_time);

  RCLCPP_INFO(get_logger(), "Evaluation complete");
  rclcpp::shutdown();
}

void AutowarePlanningDataAnalyzerNode::write_map_and_route_markers_to_bag(
  const rclcpp::Time & reference_time)
{
  if (!evaluation_bag_writer_) {
    return;
  }

  // Create and write map markers from lanelet map
  if (map_marker_) {
    // Create a copy and update timestamps to match the evaluation bag timeline
    visualization_msgs::msg::MarkerArray time_corrected_markers = *map_marker_;

    // Update all marker timestamps to the reference time
    // This ensures markers are synchronized with the TF data in the evaluation bag
    for (auto & marker : time_corrected_markers.markers) {
      marker.header.stamp = reference_time;
    }

    evaluation_bag_writer_->write(time_corrected_markers, "/map_markers", reference_time);
  }

  // Set route for route handler if available
  if (route_handler_ && route_handler_->isHandlerReady()) {
    // Create and save route markers
    visualization_msgs::msg::MarkerArray route_markers;
    create_route_markers(route_markers);
    if (!route_markers.markers.empty()) {
      evaluation_bag_writer_->write(route_markers, "/route_markers", reference_time);
    }
  }
}

void AutowarePlanningDataAnalyzerNode::create_route_markers(
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (!route_handler_ || !route_handler_->isHandlerReady()) {
    return;
  }

  const auto preferred_lanes = route_handler_->getPreferredLanelets();
  if (preferred_lanes.empty()) {
    return;
  }

  autoware_utils::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "preferred_lane", preferred_lanes, create_marker_color(0.16, 1.0, 0.69, 0.2)),
    &marker_array);
}
}  // namespace autoware::planning_data_analyzer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_data_analyzer::AutowarePlanningDataAnalyzerNode)
