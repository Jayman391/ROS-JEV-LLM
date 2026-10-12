cmake_minimum_required(VERSION 3.8)
project(line_gz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
# Gazebo Fortress is optional: images without it (jev-ai-line) build everything else
find_package(ignition-transport11 QUIET)
find_package(ignition-msgs8 QUIET)
if(NOT ignition-transport11_FOUND OR NOT ignition-msgs8_FOUND)
  message(WARNING "line_gz: Gazebo Fortress (ignition-transport11) not found; skipping")
  ament_package()
  return()
endif()

find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(line_interfaces REQUIRED)

add_executable(line_gz src/line_gz.cpp)
ament_target_dependencies(line_gz rclcpp visualization_msgs line_interfaces)
target_link_libraries(line_gz ignition-transport11::core ignition-msgs8::core)
install(TARGETS line_gz DESTINATION lib/${PROJECT_NAME})

ament_package()
