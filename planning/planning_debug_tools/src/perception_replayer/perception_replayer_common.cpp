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

#include "perception_replayer_common.hpp"

#include "utils.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::planning_debug_tools
{

std::vector<std::string> PerceptionReplayerCommon::find_rosbag_files(
  const std::string & directory_path, const std::string & rosbag_format) const
{
  const std::string extension = (rosbag_format == "mcap") ? ".mcap" : ".db3";
  std::vector<std::string> rosbag_files;

  for (const auto & entry : std::filesystem::directory_iterator(directory_path)) {
    if (entry.is_regular_file() && entry.path().extension() == extension) {
      rosbag_files.push_back(entry.path().string());
    }
  }

  // sort by modification time (or filename if you prefer)
  std::sort(rosbag_files.begin(), rosbag_files.end());

  return rosbag_files;
}

void PerceptionReplayerCommon::load_rosbag(
  const std::string & rosbag_path, const std::string & rosbag_format)
{
  std::cout << "Loading rosbag: " << rosbag_path << std::endl;

  auto reader = std::make_unique<rosbag2_cpp::Reader>();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbag_path;
  storage_options.storage_id = rosbag_format;

  reader->open(storage_options);

  // get topic metadata
  const auto topics = reader->get_all_topics_and_types();
  std::cout << "Found " << topics.size() << " topics in bag" << std::endl;

  // create type support map for deserialization
  std::unordered_map<std::string, std::shared_ptr<const rosidl_message_type_support_t>>
    type_support_map;
  std::unordered_map<std::string, std::shared_ptr<rcpputils::SharedLibrary>> type_support_libs;

  // try to load type support for each topic
  for (const auto & topic_meta : topics) {
    try {
      auto library =
        rosbag2_cpp::get_typesupport_library(topic_meta.type, "rosidl_typesupport_cpp");
      type_support_libs[topic_meta.name] = library;

      const rosidl_message_type_support_t * type_support =
        rosbag2_cpp::get_typesupport_handle(topic_meta.type, "rosidl_typesupport_cpp", library);

      if (type_support) {
        type_support_map[topic_meta.name] = std::shared_ptr<const rosidl_message_type_support_t>(
          type_support, [](const rosidl_message_type_support_t *) {});
      }
    } catch (const std::exception & e) {
      // skip topics with unknown message types
      std::cerr << "Warning: Could not load type support for topic " << topic_meta.name << " ("
                << topic_meta.type << ")" << std::endl;
    }
  }

  // topic_names
  const auto objects_topic = [&]() -> std::string {
    if (param_.detected_object) {
      return "/perception/object_recognition/detection/objects";
    } else {
      if (param_.tracked_object) {
        return "/perception/object_recognition/tracking/objects";
      } else {
        return "/perception/object_recognition/objects";
      }
    }
  }();
  const std::string ego_odom_topic = "/localization/kinematic_state";
  const std::string traffic_signals_topic = "/perception/traffic_light_recognition/traffic_signals";

  // create topic filter
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = {
    objects_topic,
    ego_odom_topic,
    traffic_signals_topic,
  };
  reader->set_filter(storage_filter);

  // read all messages
  while (reader->has_next()) {
    try {
      auto bag_message = reader->read_next();

      // deserialize the message if type support is available
      auto it = type_support_map.find(bag_message->topic_name);
      if (it != type_support_map.end() && it->second) {
        // deserialize ego_odom messages
        if (bag_message->topic_name == ego_odom_topic) {
          const auto ego_odom_msg =
            utils::deserialize_message<Odometry>(bag_message->serialized_data);
          const rclcpp::Time timestamp(bag_message->time_stamp);
          rosbag_ego_odom_data_.emplace_back(timestamp, *ego_odom_msg);
        }

        // deserialize objects messages
        if (bag_message->topic_name == objects_topic) {
          const rclcpp::Time timestamp(bag_message->time_stamp);
          if (param_.detected_object) {
            const auto objects_msg =
              utils::deserialize_message<DetectedObjects>(bag_message->serialized_data);
            rosbag_detected_objects_data_.emplace_back(timestamp, *objects_msg);
          } else if (param_.tracked_object) {
            const auto objects_msg =
              utils::deserialize_message<TrackedObjects>(bag_message->serialized_data);
            rosbag_tracked_objects_data_.emplace_back(timestamp, *objects_msg);
          } else {
            const auto objects_msg =
              utils::deserialize_message<PredictedObjects>(bag_message->serialized_data);
            rosbag_predicted_objects_data_.emplace_back(timestamp, *objects_msg);
          }
        }

        // deserialize traffic_signals messages
        if (bag_message->topic_name == traffic_signals_topic) {
          const auto traffic_signals_msg =
            utils::deserialize_message<TrafficLightGroupArray>(bag_message->serialized_data);
          const rclcpp::Time timestamp(bag_message->time_stamp);
          rosbag_traffic_signals_data_.emplace_back(timestamp, *traffic_signals_msg);
        }
      } else {
        // count messages that couldn't be deserialized
      }
    } catch (const std::exception & e) {
      std::cerr << "\nError reading message: " << e.what() << std::endl;
      continue;
    }
  }

  std::cout << "Finished loading rosbag: " << rosbag_path << std::endl;
}

PerceptionReplayerCommon::PerceptionReplayerCommon(
  const PerceptionReplayerCommonParam & param, const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), param_(param)
{
  // check if rosbag_path is a directory or file
  if (std::filesystem::is_directory(param_.rosbag_path)) {
    std::cout << "Processing rosbag directory: " << param_.rosbag_path << std::endl;
    const auto rosbag_files = find_rosbag_files(param_.rosbag_path, param_.rosbag_format);
    std::cout << "Found " << rosbag_files.size() << " bag files" << std::endl;

    for (size_t i = 0; i < rosbag_files.size(); ++i) {
      std::cout << "Loading bag file " << (i + 1) << "/" << rosbag_files.size() << ": "
                << rosbag_files[i] << std::endl;
      load_rosbag(rosbag_files[i], param_.rosbag_format);
    }
  } else {
    std::cout << "Loading single bag file: " << param_.rosbag_path << std::endl;
    load_rosbag(param_.rosbag_path, param_.rosbag_format);
  }
  std::cout << "Ended loading rosbag" << std::endl;

  // define topic names
  const std::string ego_odom_topic = "/localization/kinematic_state";
  const auto objects_topic = [&]() -> std::string {
    if (param_.detected_object) {
      return "/perception/object_recognition/detection/objects";
    } else {
      if (param_.tracked_object) {
        return "/perception/object_recognition/tracking/objects";
      } else {
        return "/perception/object_recognition/objects";
      }
    }
  }();

  // Subscriber
  ego_odom_sub_ = this->create_subscription<Odometry>(
    ego_odom_topic, 1,
    std::bind(&PerceptionReplayerCommon::on_ego_odom, this, std::placeholders::_1));

  // Publisher
  recorded_ego_pub_ = this->create_publisher<Odometry>("/perception_reproducer/rosbag_ego_odom", 1);

  // create objects publisher based on the option
  if (param_.detected_object) {
    objects_pub_ = this->create_publisher<DetectedObjects>(
      "/perception/object_recognition/detection/objects", 1);
  } else if (param_.tracked_object) {
    objects_pub_ =
      this->create_publisher<TrackedObjects>("/perception/object_recognition/tracking/objects", 1);
  } else {
    objects_pub_ =
      this->create_publisher<PredictedObjects>("/perception/object_recognition/objects", 1);
  }

  traffic_signals_pub_ = this->create_publisher<TrafficLightGroupArray>(
    "/perception/traffic_light_recognition/traffic_signals", 1);

  pointcloud_pub_ =
    this->create_publisher<PointCloud2>("/perception/obstacle_segmentation/pointcloud", 1);

  recorded_ego_as_initialpose_pub_ =
    this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 1);
  goal_as_mission_planning_goal_pub_ =
    this->create_publisher<PoseStamped>("/planning/mission_planning/goal", 1);

  // create timer to periodically check and kill online perception nodes (1 Hz)
  timer_check_perception_process_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(1),
    std::bind(&PerceptionReplayerCommon::kill_online_perception_node, this));
}

Odometry PerceptionReplayerCommon::find_ego_odom_by_timestamp(const rclcpp::Time & timestamp) const
{
  const size_t idx = utils::get_nearest_index(rosbag_ego_odom_data_, timestamp);
  return rosbag_ego_odom_data_.at(idx).second;
}

void PerceptionReplayerCommon::publish_topics_at_timestamp(
  const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp)
{
  // for debugging
  recorded_ego_pub_->publish(find_ego_odom_by_timestamp(bag_timestamp));

  // publish objects
  if (param_.detected_object) {
    const auto objects_msg =
      utils::find_message_by_timestamp(rosbag_detected_objects_data_, bag_timestamp);
    if (objects_msg.has_value()) {
      auto msg = objects_msg.value();
      msg.header.stamp = current_timestamp;

      // apply coordinate transformation if ego_odom is available
      if (ego_odom_) {
        const auto log_ego_odom = find_ego_odom_by_timestamp(bag_timestamp);
        utils::translate_objects_coordinate(ego_odom_->pose.pose, log_ego_odom.pose.pose, msg);
      }

      auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<DetectedObjects>>(objects_pub_);
      if (publisher) {
        publisher->publish(msg);
      }
    }
  } else if (param_.tracked_object) {
    const auto objects_msg =
      utils::find_message_by_timestamp(rosbag_tracked_objects_data_, bag_timestamp);
    if (objects_msg.has_value()) {
      auto msg = objects_msg.value();
      msg.header.stamp = current_timestamp;
      auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<TrackedObjects>>(objects_pub_);
      if (publisher) {
        publisher->publish(msg);
      }
    }
  } else {
    const auto objects_msg =
      utils::find_message_by_timestamp(rosbag_predicted_objects_data_, bag_timestamp);
    if (objects_msg.has_value()) {
      auto msg = objects_msg.value();
      msg.header.stamp = current_timestamp;
      auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<PredictedObjects>>(objects_pub_);
      if (publisher) {
        publisher->publish(msg);
      }
    }
  }

  publish_traffic_ligihts_at_timestamp(bag_timestamp, current_timestamp);
}

void PerceptionReplayerCommon::publish_topics_at_timestamp_with_coordinate_conversion(
  const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp,
  const Odometry & current_ego_odom)
{
  // for debugging
  recorded_ego_pub_->publish(find_ego_odom_by_timestamp(bag_timestamp));

  // publish objects
  if (param_.detected_object) {
    const auto objects_msg =
      utils::find_message_by_timestamp(rosbag_detected_objects_data_, bag_timestamp);
    if (objects_msg.has_value()) {
      auto msg = objects_msg.value();
      msg.header.stamp = current_timestamp;

      // apply coordinate transformation using provided current ego odom
      const auto log_ego_odom = find_ego_odom_by_timestamp(bag_timestamp);
      utils::translate_objects_coordinate(current_ego_odom.pose.pose, log_ego_odom.pose.pose, msg);

      auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<DetectedObjects>>(objects_pub_);
      if (publisher) {
        publisher->publish(msg);
      }
    }
  } else if (param_.tracked_object) {
    const auto objects_msg =
      utils::find_message_by_timestamp(rosbag_tracked_objects_data_, bag_timestamp);
    if (objects_msg.has_value()) {
      auto msg = objects_msg.value();
      msg.header.stamp = current_timestamp;
      auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<TrackedObjects>>(objects_pub_);
      if (publisher) {
        publisher->publish(msg);
      }
    }
  } else {
    const auto objects_msg =
      utils::find_message_by_timestamp(rosbag_predicted_objects_data_, bag_timestamp);
    if (objects_msg.has_value()) {
      auto msg = objects_msg.value();
      msg.header.stamp = current_timestamp;
      auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<PredictedObjects>>(objects_pub_);
      if (publisher) {
        publisher->publish(msg);
      }
    }
  }

  publish_traffic_ligihts_at_timestamp(bag_timestamp, current_timestamp);
}

void PerceptionReplayerCommon::publish_traffic_ligihts_at_timestamp(
  const rclcpp::Time & bag_timestamp, const rclcpp::Time & current_timestamp)
{
  const auto traffic_signals_msg =
    utils::find_message_by_timestamp(rosbag_traffic_signals_data_, bag_timestamp);
  if (traffic_signals_msg.has_value()) {
    auto msg = traffic_signals_msg.value();
    const auto original_timestamp = msg.stamp;
    msg.stamp = current_timestamp;

    for (auto & traffic_signals_group : msg.traffic_light_groups) {
      for (auto & prediction : traffic_signals_group.predictions) {
        // time difference between original stamp and prediction stamp
        const auto time_diff = rclcpp::Time(prediction.predicted_stamp) - original_timestamp;

        // fix timestamp
        prediction.predicted_stamp = current_timestamp + time_diff;
      }
    }

    traffic_signals_pub_->publish(msg);
  }
}

void PerceptionReplayerCommon::publish_recorded_ego_pose(rclcpp::Time bag_timestamp)
{
  const auto ego_odom = find_ego_odom_by_timestamp(bag_timestamp);

  PoseWithCovarianceStamped initialpose;
  initialpose.header.stamp = this->get_clock()->now();
  initialpose.header.frame_id = "map";
  initialpose.pose.pose = ego_odom.pose.pose;

  // clang-format off
  initialpose.pose.covariance = {
    0.25,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.25,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.06853892326654787,
  };
  // clang-format on

  recorded_ego_as_initialpose_pub_->publish(initialpose);

  RCLCPP_INFO(get_logger(), "Published recorded ego pose as /initialpose");
}

void PerceptionReplayerCommon::publish_goal_pose()
{
  const auto ego_pose = find_ego_odom_by_timestamp(get_bag_end_timestamp());

  PoseStamped goal_pose;
  goal_pose.header.stamp = this->get_clock()->now();
  goal_pose.header.frame_id = "map";
  goal_pose.pose = ego_pose.pose.pose;

  goal_as_mission_planning_goal_pub_->publish(goal_pose);
  RCLCPP_INFO(get_logger(), "Published last recorded ego pose as /planning/mission_planning/goal");
}

void PerceptionReplayerCommon::on_ego_odom(const Odometry::SharedPtr msg)
{
  ego_odom_ = msg;
}

void PerceptionReplayerCommon::publish_empty_pointcloud(const rclcpp::Time & current_timestamp)
{
  PointCloud2 pointcloud_msg;
  pointcloud_msg.header.stamp = current_timestamp;
  pointcloud_msg.header.frame_id = "map";
  pointcloud_msg.height = 1;
  pointcloud_msg.is_dense = true;
  pointcloud_msg.point_step = 16;
  pointcloud_msg.width = 0;
  const auto point_field = [](const std::string & name, const uint32_t offset) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    return field;
  };
  pointcloud_msg.fields = {
    point_field("x", 0),
    point_field("y", 4),
    point_field("z", 8),
  };

  pointcloud_pub_->publish(pointcloud_msg);
}

void PerceptionReplayerCommon::kill_online_perception_node()
{
  std::string kill_process_name;

  if (param_.detected_object) {
    kill_process_name = "dummy_perception_publisher_node";
  } else if (param_.tracked_object) {
    kill_process_name = "multi_object_tracker";
  } else {
    kill_process_name = "map_based_prediction";
  }

  if (kill_process_name.empty()) {
    return;
  }

  // use pidof to find the process
  const std::string command = "pidof " + kill_process_name + " 2>/dev/null";
  FILE * pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return;
  }

  char buffer[128];
  std::string result;
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  const int pclose_result = pclose(pipe);

  // check if pidof found the process (exit code 0)
  if (pclose_result == 0 && !result.empty()) {
    try {
      // parse pid from result
      const pid_t pid = static_cast<pid_t>(std::stol(result));

      // send SIGTERM (same as Python's process.terminate())
      const std::string kill_command = "kill -TERM " + std::to_string(pid) + " 2>/dev/null";
      const int kill_result = system(kill_command.c_str());

      if (kill_result == 0) {
        RCLCPP_INFO(
          get_logger(), "Terminated online perception node: %s (PID: %d)",
          kill_process_name.c_str(), pid);
      }
    } catch (const std::exception & e) {
      // failed to convert pid, ignore
    }
  }
}

}  // namespace autoware::planning_debug_tools
