#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <numbers>

#include "message_turtle_commands/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

namespace action_turtle_commands {

using namespace std::placeholders;
template<class T> using sptr = std::shared_ptr<T>;
using std::string;
using std::numbers::pi;
using std::mutex;
using std::unique_lock;
using std::bind;
using rclcpp::ok;
using rclcpp::NodeOptions;
using geometry_msgs::msg::Twist;
using turtlesim::msg::Pose;
using Action = message_turtle_commands::action::MessageTurtleCommands;
using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;

class ActionTurtleServer : public rclcpp::Node {
 public:
  explicit ActionTurtleServer(const NodeOptions & options = NodeOptions())
      : Node("action_turtle_server", options) {
    action_server_ = rclcpp_action::create_server<Action>(
        this, "message_turtle_commands",
        bind(&ActionTurtleServer::HandleGoal, this, _1, _2),
        bind(&ActionTurtleServer::HanleCancel, this, _1),
        bind(&ActionTurtleServer::HandleAccepted, this, _1));

    twist_publisher_ = create_publisher<Twist>("/turtle1/cmd_vel", 10);
    pose_subscriber_ = create_subscription<Pose>(
        "/turtle1/pose", 10,
        bind(&ActionTurtleServer::PoseCallback, this, _1));
  }

 private:
  rclcpp_action::Server<Action>::SharedPtr action_server_;

  rclcpp::Publisher<Twist>::SharedPtr twist_publisher_;

  rclcpp::Subscription<Pose>::SharedPtr pose_subscriber_;

  Pose latest_pose_;
  mutex pose_mutex_;

  const string forward_command = string("forward");
  const string turn_left_command = string("turn_left");
  const string turn_right_command = string("turn_right");

  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID & uuid,
                                         sptr<const Action::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with command %s", goal->command.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HanleCancel(
      const sptr<GoalHandleAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const sptr<GoalHandleAction> goal_handle) {
    std::thread { bind(&ActionTurtleServer::Execute, this, _1), goal_handle}
        .detach();
  }

  void Execute(const sptr<GoalHandleAction> goal_handle) {
    const auto goal = goal_handle->get_goal();

    if (goal->command == forward_command)
      ExecuteForwardCommand(goal_handle);
    else if (goal->command == turn_left_command)
      ExecuteTurnLeftCommand(goal_handle);
    else if (goal->command == turn_right_command)
      ExecuteTurnLeftCommand(goal_handle, true);
  }

  void ExecuteForwardCommand(const sptr<GoalHandleAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing movement goal");

    const auto goal = goal_handle->get_goal();

    auto start_pose = latest_pose_;

    auto twist = Twist();
    twist.linear.x = goal->distance;
    twist_publisher_->publish(twist);

    rclcpp::Rate loop_rate(1);

    auto feedback = std::make_shared<Action::Feedback>();
    auto result = std::make_shared<Action::Result>();

    {
      auto lock = unique_lock(pose_mutex_);
      feedback->traveled_distance = hypot(latest_pose_.x - start_pose.x,
                                          latest_pose_.y - start_pose.y);
    }

    while(feedback->traveled_distance < goal->distance && ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      {
        auto lock = unique_lock(pose_mutex_);
        feedback->traveled_distance = hypot(latest_pose_.x - start_pose.x,
                                            latest_pose_.y - start_pose.y);
      }
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (ok()) {
      result->is_successful = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void ExecuteTurnLeftCommand(const sptr<GoalHandleAction> goal_handle,
                              bool invert = false) {
    RCLCPP_INFO(this->get_logger(), "Executing turn goal");

    const auto goal = goal_handle->get_goal();

    auto start_pose = latest_pose_;

    auto sign = invert ? -1 : 1;

    auto twist = Twist();
    twist.angular.z = goal->angle_degrees / 180. * pi * sign;
    RCLCPP_INFO(this->get_logger(), "ABOBA %lf", twist.angular.z);
    twist_publisher_->publish(twist);

    rclcpp::Rate loop_rate(1);

    auto feedback = std::make_shared<Action::Feedback>();
    auto result = std::make_shared<Action::Result>();

    {
      auto lock = unique_lock(pose_mutex_);
      feedback->traveled_distance = (latest_pose_.theta - start_pose.theta) /
                                     pi * 180. * sign;
    }

    while(feedback->traveled_distance < goal->angle_degrees && ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      {
        auto lock = unique_lock(pose_mutex_);
        feedback->traveled_distance = (latest_pose_.theta - start_pose.theta) /
                                       pi * 180. * sign;
      }
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

    loop_rate.sleep();
    }

    if (ok()) {
      result->is_successful = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void PoseCallback(const Pose & pose) {
    auto lock = unique_lock(pose_mutex_);
    latest_pose_ = pose;
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_commands::ActionTurtleServer)