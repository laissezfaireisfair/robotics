cmake_minimum_required(VERSION 3.8)
project(text_to_cmd_vel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(text_to_cmd_vel src/text_to_cmd_vel.cpp)
ament_target_dependencies(text_to_cmd_vel rclcpp std_msgs geometry_msgs)

install(TARGETS
  text_to_cmd_vel
  DESTINATION lib/${PROJECT_NAME})

ament_package()