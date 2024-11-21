#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <queue>

#include "message_turtle_commands/action/message_turtle_commands.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_turtle_commands {

using std::queue;
using namespace std::placeholders;
template <class T> using sptr=std::shared_ptr<T>;
using Action = message_turtle_commands::action::MessageTurtleCommands;
using GoalHandleAction = rclcpp_action::ClientGoalHandle<Action>;

class ActionTurtleClient : public rclcpp::Node {
 public:
  explicit ActionTurtleClient(const rclcpp::NodeOptions & options)
      : Node("action_turtle_client", options) {
    client_ptr_ = rclcpp_action::create_client<Action>(
        this, "message_turtle_commands");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ActionTurtleClient::SendGoal, this));

    goals_queue_.push(ConstructGoal(std::string("forward"), 2, 0));
    goals_queue_.push(ConstructGoal(std::string("turn_right"), 0, 90));
    goals_queue_.push(ConstructGoal(std::string("forward"), 1, 0));
  }

  void SendGoal() {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = goals_queue_.front();
    goals_queue_.pop();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionTurtleClient::GoalResponseCallback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionTurtleClient::FeedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionTurtleClient::ResultCallback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<Action>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  queue<Action::Goal> goals_queue_;

  void GoalResponseCallback(const sptr<GoalHandleAction> & goal_handle) {
    if (!goal_handle)
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    else
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
  }

  void FeedbackCallback(sptr<GoalHandleAction>,
                        const sptr<const Action::Feedback> feedback) {
    std::stringstream ss;
    ss << "Feedback received: " << feedback->traveled_distance;

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void ResultCallback(const GoalHandleAction::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::stringstream ss;
    ss << "Result received: " << result.result->is_successful;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    if (!goals_queue_.empty()) {
      SendGoal();
      return;
    }

    rclcpp::shutdown();
  }

  Action::Goal ConstructGoal(std::string command, int distance,
                             int angle_degrees) {
    auto goal_msg = Action::Goal();
    goal_msg.command = command;
    goal_msg.distance = distance;
    goal_msg.angle_degrees = angle_degrees;
    return goal_msg;
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_commands::ActionTurtleClient)