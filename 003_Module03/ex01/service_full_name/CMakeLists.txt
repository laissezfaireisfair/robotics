cmake_minimum_required(VERSION 3.8)
project(service_full_name)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(names_interface REQUIRED)

add_executable(service_name src/concat_full_name.cpp)
ament_target_dependencies(service_name rclcpp names_interface)

add_executable(client_name src/client.cpp)
ament_target_dependencies(client_name rclcpp names_interface)

install(TARGETS
  service_name
  client_name
  DESTINATION lib/${PROJECT_NAME})

ament_package()