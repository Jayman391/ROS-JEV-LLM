cmake_minimum_required(VERSION 3.8)
project(swarm_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(jev_ai_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)

set(DEPS rclcpp rclcpp_action geometry_msgs nav_msgs sensor_msgs std_msgs diagnostic_msgs
  jev_ai_msgs swarm_interfaces)

foreach(node flight_controller sensor_sim comms_sim formation_planner)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE include)
  ament_target_dependencies(${node} ${DEPS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_world test/test_world.cpp)
  target_include_directories(test_world PRIVATE include)
  ament_target_dependencies(test_world rclcpp)
endif()

ament_package()
