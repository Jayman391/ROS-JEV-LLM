cmake_minimum_required(VERSION 3.8)
project(jev_ai_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(jev_ai_msgs REQUIRED)

add_library(jev_ai_cpp SHARED src/clients.cpp)
target_include_directories(jev_ai_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(jev_ai_cpp rclcpp rclcpp_action diagnostic_msgs jev_ai_msgs)

# component example: the C++ toy_patrol agent
add_library(toy_agent_component SHARED examples/toy_agent.cpp)
target_link_libraries(toy_agent_component jev_ai_cpp)
ament_target_dependencies(toy_agent_component rclcpp rclcpp_components)
rclcpp_components_register_node(toy_agent_component PLUGIN "jev_ai_cpp::ToyAgent" EXECUTABLE toy_agent)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS jev_ai_cpp EXPORT export_jev_ai_cpp
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS toy_agent_component LIBRARY DESTINATION lib)

ament_export_targets(export_jev_ai_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action diagnostic_msgs jev_ai_msgs)
ament_package()
