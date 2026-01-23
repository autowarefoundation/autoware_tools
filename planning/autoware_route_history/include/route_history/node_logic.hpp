// Copyright 2026 TIER IV, Inc.
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

#ifndef ROUTE_HISTORY__NODE_LOGIC_HPP_
#define ROUTE_HISTORY__NODE_LOGIC_HPP_

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <exception>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::route_history
{

using adapi_route = autoware_adapi_v1_msgs::msg::Route;

struct NamedRoute
{
  std::string name;
  adapi_route route;
};

struct UuidName
{
  std::string uuid;
  std::string name;
};

using uuid_route_map = std::unordered_map<std::string, NamedRoute>;

class NodeLogic
{
public:
  explicit NodeLogic(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    if (!node_->has_parameter("save_file_path")) {
      node_->declare_parameter("save_file_path", "~/.ros/route_history.yaml");
    }
    save_file_cb_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : parameters) {
          if (p.get_name() == "save_file_path") {
            std::string path = expand_home_path(p.get_value<std::string>());
            this->read_routes(path);
          }
        }
        return result;
      });

    route_set_subscription_ = node_->create_subscription<adapi_route>(
      "/api/routing/route", 10, [this](const adapi_route & msg) { route_set_callback(msg); });

    initial_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    goal_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 10);
    sync_notif_publisher_ = node_->create_publisher<std_msgs::msg::String>("update", 10);

    read_routes(get_save_path());
  }

  ~NodeLogic()
  {
    save_file_cb_.reset();
    route_set_subscription_.reset();
    initial_pose_publisher_.reset();
    goal_pose_publisher_.reset();
    sync_notif_publisher_.reset();

    RCLCPP_INFO(node_->get_logger(), "NogeLogic clean up successfull.");
  }

  auto get_save_path() -> std::string
  {
    return expand_home_path(node_->get_parameter("save_file_path").as_string());
  }

  void set_save_path(std::string new_path)
  {
    node_->set_parameters({rclcpp::Parameter("save_file_path", new_path)});
  }

  auto expand_home_path(const std::string & path) -> std::string
  {
    if (!path.empty() && path[0] == '~') {
      const char * home = getenv("HOME");
      if (home) {
        return std::string(home) + path.substr(1);
      }
    }
    return path;
  }

  auto get_routes(const std::vector<std::string> & uuids) -> std::vector<UuidName>
  {
    std::vector<UuidName> fetched_routes;
    if (uuids.size() == 0) {
      for (auto & [key, value] : routes) {
        UuidName res;
        res.uuid = key;
        res.name = value.name;
        fetched_routes.push_back(res);
      }
    } else {
      for (auto & uuid : uuids) {
        if (routes.count(uuid)) {
          auto route = routes[uuid];
          UuidName res;
          res.uuid = uuid;
          res.name = route.name;
          fetched_routes.push_back(res);
        } else {
          RCLCPP_INFO(
            node_->get_logger(), "[get_routes] No route found in save file for uuid %s.",
            uuid.c_str());
        }
      }
    }
    return fetched_routes;
  }

  void load_route(const std::string & uuid)
  {
    if (uuid.size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "[set_route] No uuid given.");
      return;
    }

    try {
      geometry_msgs::msg::PoseWithCovarianceStamped initialmsg;

      initialmsg.header.frame_id = routes.at(uuid).route.header.frame_id;

      initialmsg.pose.pose.position.x = routes.at(uuid).route.data[0].start.position.x;
      initialmsg.pose.pose.position.y = routes.at(uuid).route.data[0].start.position.y;
      initialmsg.pose.pose.position.z = routes.at(uuid).route.data[0].start.position.z;

      initialmsg.pose.pose.orientation.x = routes.at(uuid).route.data[0].start.orientation.x;
      initialmsg.pose.pose.orientation.y = routes.at(uuid).route.data[0].start.orientation.y;
      initialmsg.pose.pose.orientation.z = routes.at(uuid).route.data[0].start.orientation.z;
      initialmsg.pose.pose.orientation.w = routes.at(uuid).route.data[0].start.orientation.w;

      geometry_msgs::msg::PoseStamped goalmsg;

      goalmsg.header.frame_id = routes.at(uuid).route.header.frame_id;

      goalmsg.pose.position.x = routes.at(uuid).route.data[0].goal.position.x;
      goalmsg.pose.position.y = routes.at(uuid).route.data[0].goal.position.y;
      goalmsg.pose.position.z = routes.at(uuid).route.data[0].goal.position.z;

      goalmsg.pose.orientation.x = routes.at(uuid).route.data[0].goal.orientation.x;
      goalmsg.pose.orientation.y = routes.at(uuid).route.data[0].goal.orientation.y;
      goalmsg.pose.orientation.z = routes.at(uuid).route.data[0].goal.orientation.z;
      goalmsg.pose.orientation.w = routes.at(uuid).route.data[0].goal.orientation.w;

      initial_pose_publisher_->publish(initialmsg);
      goal_pose_publisher_->publish(goalmsg);
    } catch (const YAML::BadFile & e) {
      RCLCPP_INFO(
        node_->get_logger(), "[set_route] Save file not found at path: %s.",
        get_save_path().c_str());
    } catch (const YAML::BadConversion & e) {
      RCLCPP_INFO(node_->get_logger(), "[set_route] Bad conversion, yaml to pose: %s", e.what());
    }
  }

  void delete_route(const std::string & uuid)
  {
    if (uuid.size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "[delete_route] No uuid given.");
      return;
    }

    std::ifstream yaml_file(get_save_path());

    // Read and save to function scope
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
    yaml_file.close();

    // Delete from function scope
    for (auto p = docs.begin(); p != docs.end();) {
      if (!*p || !(*p)["uuid"]) {
        ++p;
        continue;
      }
      if ((*p)["uuid"].as<std::string>() == uuid) {
        p = docs.erase(p);
        RCLCPP_INFO(
          node_->get_logger(), "[delete_route] Deleted route with uuid: %s.", uuid.c_str());
      } else {
        ++p;
      }
    }

    // Wipe file and re-write to save
    clear_file(get_save_path());
    for (auto & d : docs) {
      write_route(get_save_path(), d, true);
    }

    // Update local node to sync
    read_routes(get_save_path());
  }

  void set_name(const std::string & uuid, const std::string & new_name)
  {
    if (uuid.size() == 0 || new_name.size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "[set_name] Invalid empty uuid or name given.");
      return;
    }

    std::ifstream yaml_file(get_save_path());

    // Read and save to function scope
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
    yaml_file.close();

    // Delete from function scope
    for (auto p = docs.begin(); p != docs.end();) {
      if (!*p || !(*p)["uuid"]) {
        ++p;
        continue;
      }
      if ((*p)["uuid"].as<std::string>() == uuid) {
        (*p)["name"] = new_name;
        RCLCPP_INFO(
          node_->get_logger(), "[set_name] Name changed to \"%s\" for route with uuid: %s.",
          new_name.c_str(), uuid.c_str());
        p = docs.end();
      } else {
        ++p;
      }
    }

    // Wipe file and re-write to save
    clear_file(get_save_path());
    for (auto & d : docs) {
      write_route(get_save_path(), d, true);
    }

    // Update local node to sync
    read_routes(get_save_path());
  }

  void clear_file(const std::string & filepath)
  {
    std::ofstream o;
    o.open(filepath, std::ios::trunc);
  }

  void clear_routes() { routes = {}; }

  template <typename T>
  void write_route(const std::string & filepath, T & value, bool append = true)
  {
    std::ofstream o;
    if (append) {
      o.open(filepath, std::ios::app);
    } else {
      o.open(filepath, std::ios::trunc);
    }

    if (!o.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "[write_route] Cannot open file: %s.", filepath.c_str());
      throw std::runtime_error("[write_route] Cannot open file: " + filepath + ".");
    }

    if (!o.is_open()) throw std::runtime_error("[write_route] Cannot open file: " + filepath + ".");
    o << "---\n" << value << "\n";
    o.close();
  }

  auto read_yaml_to_node_vector(const std::string & filepath) -> std::vector<YAML::Node>
  {
    std::ifstream yaml_file(filepath);
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
    if (docs.empty()) return {};
    return docs;
  }

  void read_routes(const std::string & filepath)
  {
    std::vector<YAML::Node> docs = read_yaml_to_node_vector(filepath);
    clear_routes();
    for (auto & doc : docs) {
      append_route(doc);
    }
  }

  void append_route(const YAML::Node & node)
  {
    uuid_route_map new_map = yaml_to_map(node);
    routes.insert(new_map.begin(), new_map.end());
  }

  auto prepend_uuid_name(const std::string & yaml_str) -> std::string
  {
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid_str = gen();

    std::ostringstream oss;
    oss << "name: \"New route"
        << "\"\n";
    oss << "uuid: \"" << uuid_str << "\"\n";
    oss << yaml_str;
    std::string new_str = oss.str();
    return new_str;
  }

  void route_set_callback(const adapi_route & msg) { current_route = msg; }

  void save_route()
  {
    if (current_route.data.empty()) return;

    std::string yaml_str = autoware_adapi_v1_msgs::msg::to_yaml(current_route);

    auto uuid_yaml_str = prepend_uuid_name(yaml_str);
    write_route(get_save_path(), uuid_yaml_str, true);
    RCLCPP_INFO(
      node_->get_logger(), "[route_set_callback] Route written to %s.", get_save_path().c_str());
    read_routes(get_save_path());
  }

  auto yaml_to_map(YAML::Node root) -> uuid_route_map
  {
    autoware_adapi_v1_msgs::msg::Route msg;

    // Parse header
    auto header_node = root["header"];
    msg.header.stamp.sec = header_node["stamp"]["sec"].as<int32_t>();
    msg.header.stamp.nanosec = header_node["stamp"]["nanosec"].as<uint32_t>();
    msg.header.frame_id = header_node["frame_id"].as<std::string>();

    // Parse data (array, max 1)
    auto data_node = root["data"];
    if (data_node && data_node.IsSequence() && !data_node.IsNull()) {
      for (const auto & route_node : data_node) {
        autoware_adapi_v1_msgs::msg::RouteData route_data;

        // Start pose
        route_data.start.position.x = route_node["start"]["position"]["x"].as<double>();
        route_data.start.position.y = route_node["start"]["position"]["y"].as<double>();
        route_data.start.position.z = route_node["start"]["position"]["z"].as<double>();

        route_data.start.orientation.x = route_node["start"]["orientation"]["x"].as<double>();
        route_data.start.orientation.y = route_node["start"]["orientation"]["y"].as<double>();
        route_data.start.orientation.z = route_node["start"]["orientation"]["z"].as<double>();
        route_data.start.orientation.w = route_node["start"]["orientation"]["w"].as<double>();

        // Goal pose
        route_data.goal.position.x = route_node["goal"]["position"]["x"].as<double>();
        route_data.goal.position.y = route_node["goal"]["position"]["y"].as<double>();
        route_data.goal.position.z = route_node["goal"]["position"]["z"].as<double>();

        route_data.goal.orientation.x = route_node["goal"]["orientation"]["x"].as<double>();
        route_data.goal.orientation.y = route_node["goal"]["orientation"]["y"].as<double>();
        route_data.goal.orientation.z = route_node["goal"]["orientation"]["z"].as<double>();
        route_data.goal.orientation.w = route_node["goal"]["orientation"]["w"].as<double>();

        // Segments
        auto segments_node = route_node["segments"];
        if (segments_node && segments_node.IsSequence()) {
          for (const auto & seg_node : segments_node) {
            autoware_adapi_v1_msgs::msg::RouteSegment seg;

            // Preferred
            seg.preferred.id = seg_node["preferred"]["id"].as<int64_t>();
            seg.preferred.type = seg_node["preferred"]["type"].as<std::string>();

            // Alternatives
            auto alt_node = seg_node["alternatives"];
            if (alt_node && alt_node.IsSequence()) {
              for (const auto & alt : alt_node) {
                autoware_adapi_v1_msgs::msg::RoutePrimitive alt_prim;
                alt_prim.id = alt["id"].as<int64_t>();
                alt_prim.type = alt["type"].as<std::string>();
                seg.alternatives.push_back(alt_prim);
              }
            }

            route_data.segments.push_back(seg);
          }
        }

        msg.data.push_back(route_data);
      }
    }

    uuid_route_map map_obj = {
      {root["uuid"].as<std::string>(), {root["name"].as<std::string>(), msg}}};
    return map_obj;
  }

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_notif_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr save_file_cb_;

  uuid_route_map routes;
  adapi_route current_route;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__NODE_LOGIC_HPP_
