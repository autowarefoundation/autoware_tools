#include <memory>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>

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

	};
	
	void timer_callback()
	{
		set_route();
	}
	
	void set_route()
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

	YAML::Node read_yaml(std::string filepath)
	{
		YAML::Node yaml = YAML::LoadFile(filepath);
		return yaml;
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

	void route_set_callback(const autoware_adapi_v1_msgs::msg::Route& msg)
	{	
		if(msg.data.empty()) return;
		route_to_yaml(msg);
		RCLCPP_INFO(this->get_logger(), "Route written to output.yaml.");
	}

	void route_to_yaml(const autoware_adapi_v1_msgs::msg::Route& msg){
		std::string yaml_str = autoware_adapi_v1_msgs::msg::to_yaml(msg);
		std::string uuid_yaml_str = prepend_uuid(yaml_str);	
		std::ofstream o;
		o.open("output.yaml", std::ios::app);
		if(o.is_open()) o << uuid_yaml_str << "---\n";	
		o.close();
	}	
	

	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;	
//	rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;	

};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
