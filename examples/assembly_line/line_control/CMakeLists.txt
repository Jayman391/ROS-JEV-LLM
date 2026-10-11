cmake_minimum_required(VERSION 3.8)
project(line_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(line_interfaces REQUIRED)

add_executable(line_sim src/line_sim.cpp)
target_include_directories(line_sim PRIVATE include)
ament_target_dependencies(line_sim rclcpp rclcpp_action rosgraph_msgs diagnostic_msgs
  visualization_msgs line_interfaces)
install(TARGETS line_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_line test/test_line.cpp)
  target_include_directories(test_line PRIVATE include)
endif()

ament_package()
