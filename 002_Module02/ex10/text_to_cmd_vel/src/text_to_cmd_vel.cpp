#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
template<class T> using sptr=std::shared_ptr<T>;
using String=std_msgs::msg::String;
using Twist=geometry_msgs::msg::Twist;
using Vector3=geometry_msgs::msg::Vector3;

class TextToCmdVel : public rclcpp::Node {
 public:
  TextToCmdVel() : Node("text_to_cmd_vel") {
    publisher_ = this->create_publisher<Twist>("/turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<String>( "cmd_text",
    	10, std::bind(&TextToCmdVel::TopicCallback, this, _1));
    InitializeTwistByCommand();
  }

 private:
  void InitializeTwistByCommand() {
  	auto right_twist = Twist();
  	right_twist.angular.z = -1.5;
  	twist_by_command_["turn_right"] = right_twist;

    auto left_twist = Twist();
    left_twist.angular.z = 1.5;
    twist_by_command_["turn_left"] = left_twist;

    auto forward_twist = Twist();
    forward_twist.linear.x = 1;
    twist_by_command_["move_forward"] = forward_twist;

    auto backward_twist = Twist();
    backward_twist.linear.x = -1;
    twist_by_command_["move_backward"] = backward_twist;
  }

  void TopicCallback(const String & msg) {
  	RCLCPP_INFO(this->get_logger(), "Received command: '%s'", msg.data.c_str());

  	if (!twist_by_command_.contains(msg.data)) {
  		RCLCPP_INFO(this->get_logger(), "Command unknown");
  		return;
  	}

  	auto twist = twist_by_command_[msg.data];
    publisher_->publish(twist);
  }

  rclcpp::Subscription<String>::SharedPtr subscription_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  std::unordered_map<std::string, Twist> twist_by_command_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextToCmdVel>());
  rclcpp::shutdown();
  return 0;
}