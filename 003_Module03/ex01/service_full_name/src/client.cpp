#include "rclcpp/rclcpp.hpp"
#include "names_interface/srv/summ_full_name.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
template <class T> using sptr=std::shared_ptr<T>;
using rclcpp::get_logger;
using SummFullName=names_interface::srv::SummFullName;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(get_logger("rclcpp"), "usage: client_name X Y Z");
      return 1;
  }

  auto node = rclcpp::Node::make_shared("client_name");
  auto client = node->create_client<SummFullName>("SummFullName");

  auto request = std::make_shared<SummFullName::Request>();
  request->name = argv[1];
  request->middle_name = argv[2];
  request->last_name = argv[3];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(get_logger("rclcpp"), "Full name: %s",
                result.get()->full_name.c_str());
  else
    RCLCPP_ERROR(get_logger("rclcpp"), "Failed to call service SummFullName");

  rclcpp::shutdown();
  return 0;
}