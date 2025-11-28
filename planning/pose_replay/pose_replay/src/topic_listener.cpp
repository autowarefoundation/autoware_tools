#include <memory>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <uuid/uuid.h>

using namespace std::literals::chrono_literals;
using autoware_adapi_v1_msgs::msg::to_yaml;

struct RouteWithUuid {
	std::string uuid;
	autoware_adapi_v1_msgs::msg::Route route;
	
	bool isEmpty() const {
		return uuid.empty();
	}
};

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {		
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 
		
		route_set_subscription_ = this->create_subscription<autoware_adapi_v1_msgs::msg::Route>(
			"/api/routing/route", 10, std::bind(&TopicListener::route_set_callback, this, std::placeholders::_1));
		
		initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
		goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);				
	//	timer_ = this->create_wall_timer(1s, std::bind(&TopicListener::timer_callback, this));
	//	client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/api/routing/clear_route");
		
	//	read_yaml_history_from_file("output.yaml");
	//	test_write_to_history();
	};

/*	
	void timer_callback()
	{
		set_route();
	}
*/
	
/*	void set_route(std::string uuid)
	{	
		try {
			auto yaml = read_yaml("output.yaml");
			
			geometry_msgs::msg::PoseWithCovarianceStamped msg;

			msg.header.frame_id = yaml["header"]["frame_id"].as<std::string>();

			msg.pose.pose.position.x = yaml["data"][0]["start"]["position"]["x"].as<double>();
			msg.pose.pose.position.y = yaml["data"][0]["start"]["position"]["y"].as<double>();
			msg.pose.pose.position.z = yaml["data"][0]["start"]["position"]["z"].as<double>();

			msg.pose.pose.orientation.x = yaml["data"][0]["start"]["orientation"]["x"].as<double>();
			msg.pose.pose.orientation.y = yaml["data"][0]["start"]["orientation"]["y"].as<double>();
			msg.pose.pose.orientation.z = yaml["data"][0]["start"]["orientation"]["z"].as<double>();
			msg.pose.pose.orientation.w = yaml["data"][0]["start"]["orientation"]["w"].as<double>();
			
			if (yaml["covariance"] && yaml["covariance"].size() == 36) {
			    for (size_t i = 0; i < 36; i++) {
				msg.pose.covariance[i] = yaml["covariance"][i].as<double>();
			    }
			} else {
			    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
			}
			
			geometry_msgs::msg::PoseStamped goalmsg;			
			
			goalmsg.header.frame_id = yaml["header"]["frame_id"].as<std::string>();

			goalmsg.pose.position.x = yaml["data"][0]["goal"]["position"]["x"].as<double>();
			goalmsg.pose.position.y = yaml["data"][0]["goal"]["position"]["y"].as<double>();
			goalmsg.pose.position.z = yaml["data"][0]["goal"]["position"]["z"].as<double>();

			goalmsg.pose.orientation.x = yaml["data"][0]["goal"]["orientation"]["x"].as<double>();
			goalmsg.pose.orientation.y = yaml["data"][0]["goal"]["orientation"]["y"].as<double>();
			goalmsg.pose.orientation.z = yaml["data"][0]["goal"]["orientation"]["z"].as<double>();
			goalmsg.pose.orientation.w = yaml["data"][0]["goal"]["orientation"]["w"].as<double>();

			initial_pose_publisher_->publish(msg);
			goal_pose_publisher_->publish(goalmsg);			

		} catch(const YAML::BadFile& e) {
			RCLCPP_INFO(this->get_logger(), "Output.yaml not found.");
		} catch(const YAML::BadConversion& e) {
			RCLCPP_INFO(this->get_logger(), "Bad conversion: %s", e.what());
		}

		
	}
*/
	
	RouteWithUuid yaml_to_obj(YAML::Node root){
		
		autoware_adapi_v1_msgs::msg::Route msg;

		// Parse header
		auto header_node = root["header"];

		msg.header.stamp.sec = header_node["stamp"]["sec"].as<int32_t>();
		msg.header.stamp.nanosec = header_node["stamp"]["nanosec"].as<uint32_t>();
		msg.header.frame_id = header_node["frame_id"].as<std::string>();

		// Parse data (array, max 1)
		auto data_node = root["data"];
		if (data_node && data_node.IsSequence() && !data_node.IsNull())
		{
		    for (const auto &route_node : data_node)
		    {
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
			if (segments_node && segments_node.IsSequence())
			{
			    for (const auto &seg_node : segments_node)
			    {
				autoware_adapi_v1_msgs::msg::RouteSegment seg;

				// Preferred
				seg.preferred.id = seg_node["preferred"]["id"].as<int64_t>();
				seg.preferred.type = seg_node["preferred"]["type"].as<std::string>();

				// Alternatives
				auto alt_node = seg_node["alternatives"];
				if (alt_node && alt_node.IsSequence())
				{
				    for (const auto &alt : alt_node)
				    {
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
		
		RouteWithUuid obj;
		obj.uuid = root["uuid"].as<std::string>();
		obj.route = msg;

		return obj;
	}

	
	void read_yaml_history_from_file(std::string filepath)
	{	
		// ...LoadAll
		std::ifstream yaml_file(filepath);
		std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
		
		if (docs.empty()) {
			RCLCPP_INFO(this->get_logger(), "Empty route");
			return;
		}

		std::cout << docs.size() << std::endl;

		// check for empty
		for (size_t i = 0; i < docs.size(); ++i) {
//			std::cout << YAML::Dump(docs[i]) << std::endl;
			routes_.push_back(yaml_to_obj(docs[i]));
		} 
	}

	std::string prepend_uuid(const std::string yaml_str){	
		uuid_t binuuid;
		uuid_generate(binuuid);
		char uuid_str[37];
		uuid_unparse_lower(binuuid, uuid_str);
		
		std::ostringstream oss;
		oss << "uuid: \"" << uuid_str << "\"\n";
		oss << yaml_str;
		std::string new_str = oss.str();

		return new_str;
	}

	std::string history_to_string(RouteWithUuid& obj){
		std::ostringstream oss;
		oss << "uuid: \"" << obj.uuid << "\"\n";
		oss << autoware_adapi_v1_msgs::msg::to_yaml(obj.route);
		std::string new_str = oss.str();
		return new_str;
	};

	void route_set_callback(const autoware_adapi_v1_msgs::msg::Route& msg)
	{	
		if(msg.data.empty()) return;
		std::string yaml_str = autoware_adapi_v1_msgs::msg::to_yaml(msg);
		write_to_history(yaml_str, false, "output.yaml");
		RCLCPP_INFO(this->get_logger(), "Route written to output.yaml.");
	}
	
	void test_write_to_history(){
		std::vector<std::string> yaml_strs = {};
		for(auto &r : routes_){
			std::string s = history_to_string(r);
			yaml_strs.push_back(s);
		}
		
		for(auto &ys : yaml_strs){
			write_to_history(ys, true, "new_output.yaml");
		}	
	} 

	void write_to_history(std::string& yaml_str, bool hasUuid, const std::string filepath){
		std::string uuid_yaml_str = (hasUuid) ? yaml_str : prepend_uuid(yaml_str);	
		std::ofstream o;
		o.open(filepath, std::ios::app);
		if(o.is_open()) o << "---\n" << uuid_yaml_str << "\n";	
		o.close();
	}	
	

	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;	
//	rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;	
		
	std::vector<RouteWithUuid> routes_; 
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
