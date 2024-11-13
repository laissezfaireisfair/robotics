#include "rclcpp/rclcpp.hpp"
#include "names_interface/srv/summ_full_name.hpp"

#include <memory>

template <class T> using sptr = std::shared_ptr<T>;
using rclcpp::get_logger;
using SummFullName=names_interface::srv::SummFullName;

void ConcatFullName(const sptr<SummFullName::Request> request,
                    sptr<SummFullName::Response> response) {
  response->full_name = request->name + " " + request->middle_name + " " +
                        request->last_name;
  RCLCPP_INFO(get_logger("rclcpp"),
              "response: [%s]", response->full_name.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("service_name");

  auto service = node->create_service<SummFullName>("SummFullName",
                                                  &ConcatFullName);

  RCLCPP_INFO(get_logger("rclcpp"), "Initialized");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
