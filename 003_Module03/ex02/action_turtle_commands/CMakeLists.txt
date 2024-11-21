cmake_minimum_required(VERSION 3.8)
project(action_turtle_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_turtle_commands REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_library(action_turtle_server SHARED
  src/action_turtle_server.cpp)

target_include_directories(action_turtle_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(action_turtle_server
  PRIVATE "ACTION_TURTLE_SERVER_BUILDING_DLL")

ament_target_dependencies(action_turtle_server
  "message_turtle_commands"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "turtlesim")

rclcpp_components_register_node(action_turtle_server
  PLUGIN "action_turtle_commands::ActionTurtleServer"
  EXECUTABLE action_turtle_server_exec)

install(TARGETS
  action_turtle_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_turtle_client SHARED
  src/action_turtle_client.cpp)

target_include_directories(action_turtle_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(action_turtle_client
  PRIVATE "ACTION_TURTLE_CLIENT_BUILDING_DLL")

ament_target_dependencies(action_turtle_client
  "message_turtle_commands"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

rclcpp_components_register_node(action_turtle_client
  PLUGIN "action_turtle_commands::ActionTurtleClient"
  EXECUTABLE action_turtle_client_exec)

install(TARGETS
  action_turtle_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
