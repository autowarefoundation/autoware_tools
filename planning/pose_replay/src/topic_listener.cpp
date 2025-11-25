#include <memory>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include <chrono>

using namespace std::literals::chrono_literals;

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {		
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 
		
		route_set_subscription_ = this->create_subscription<autoware_adapi_v1_msgs::msg::Route>(
			"/api/routing/route", 10, std::bind(&TopicListener::test_callback, this, std::placeholders::_1));
		
		/*	
		client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/api/routing/clear_route");
		*/
		
	        // timer_ = this->create_wall_timer(2s, std::bind(&TopicListener::timer_callback, this));	

		// route_set_publisher_ = this->create_publisher<autoware_adapi_v1_msgs::msg::Route>("api/routing/route", 10);
 
		/* test_topic_two_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"planning/mission_planning/checkpoint", 10, std::bind(&TopicListener::test_two_callback, this, std::placeholders::_1)); */
	};
	

	private:
	/*
	void timer_callback()
	{	
		RCLCPP_INFO(this->get_logger(), "Attempting route clear...");
		
		auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ClearRoute::Request>();
		auto future = client_->async_send_request(request);
		
		if(future.valid() && future.wait_for(0s) == std::future_status::ready){
			auto response = future.get();
			if(response->status.success) {RCLCPP_INFO(this->get_logger(), "Successful clear");}
			else {RCLCPP_WARN(this->get_logger(), "Failed to clear route");}
		}

		
	}*/

	void test_callback(const autoware_adapi_v1_msgs::msg::Route& msg)
	{	
		if(msg.data.empty()) return;
		
		auto data = msg.data[0];	

		double xi = data.start.position.x;
		double yi = data.start.position.y;
		double zi = data.start.position.z;

		double xg = data.goal.position.x;
		double yg = data.goal.position.y;
		double zg = data.goal.position.z;
		
		nlohmann::json j = pose_to_json(xi, yi, zi);
		fwrite_json(j);
	
		set_pose(initial_pose, xi, yi, zi);
		set_pose(initial_pose, xg, yg, zg);
			
		log_pose_update();	

		RCLCPP_INFO(this->get_logger(), "Route log start: %f", msg.data[0].start.position.x);
		RCLCPP_INFO(this->get_logger(), "Route log end: %f", msg.data[0].goal.position.x);
	}
	
	void test_two_callback(const geometry_msgs::msg::PoseStamped& msg)
	{	
		RCLCPP_INFO_ONCE(this->get_logger(), "checkpoint route log x: %f", msg.pose.position.x);
	}

	void log_pose_update(){
		std::string initialstr = "initial: ";
		std::string word;
		for(auto& v : initial_pose){
			std::stringstream ss;
			ss << v;
			initialstr += " ";
			ss >> word;
			initialstr += word;
		}	
		RCLCPP_INFO_STREAM(this->get_logger(), initialstr); 

		std::string goalstr = "goal: ";
		for(auto& v : goal_pose){
			std::stringstream ss;
			ss << v;
			goalstr += " ";
			ss >> word;
			goalstr += word;
		}	
		RCLCPP_INFO_STREAM(this->get_logger(), goalstr); 
	};
	
	nlohmann::json pose_to_json(double x, double y, double z) 
	{
		nlohmann::json j;
		j["uuid"] = 1;
		j["type"] = "initial / goal";
		j["pose"]["x"] = x;
		j["pose"]["y"] = y;
		j["pose"]["z"] = z;
		return j;
	}

	void fwrite_json(nlohmann::json j) 
	{
		std::ofstream o;
		o.open("output.json", std::ios::app);
		if(o.is_open()) o << j.dump(4);	
		o.close();
	};

	void set_pose(double (&ref)[3], double x, double y, double z) 
	{
		ref[0] = x;
		ref[1] = y;
		ref[2] = z;	
	};
	
	
	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;	
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr test_topic_two_subscription_;	
	rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	double initial_pose[3]; 
	double goal_pose[3]; 

};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
