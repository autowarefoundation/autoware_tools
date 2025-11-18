#include <memory>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 

		// /initialpose for initial pose
		initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", 10, std::bind(&TopicListener::initial_pose_topic_callback, this, std::placeholders::_1));

		// /planning/mission_planning/goal for goal - currently this includes checkpoints, need to distinguish
		goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"planning/mission_planning/goal", 10, std::bind(&TopicListener::goal_pose_topic_callback, this, std::placeholders::_1));
	};	

	private:
	void initial_pose_topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)  
	{
		double x = msg.pose.pose.position.x;
		double y = msg.pose.pose.position.y;
		double z = msg.pose.pose.position.z;
		
		nlohmann::json j = pose_to_json(x, y, z);
		fwrite_json(j);
	
		set_pose(initial_pose, x, y, z);
		
		log_pose_update();	
	}
	
	void goal_pose_topic_callback(const geometry_msgs::msg::PoseStamped& msg) 
	{
		double x = msg.pose.position.x; 
		double y = msg.pose.position.y;
		double z = msg.pose.position.z;
		
		nlohmann::json j = pose_to_json(x, y, z);
		fwrite_json(j);

		set_pose(goal_pose, x, y, z);
			
		log_pose_update();	
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
		std::ofstream o("output.json");
		o << j.dump(4);	
	};

	void set_pose(double (&ref)[3], double x, double y, double z) 
	{
		ref[0] = x;
		ref[1] = y;
		ref[2] = z;	
	};
	
	
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;

	double initial_pose[3]; 
	double goal_pose[3]; 

};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
