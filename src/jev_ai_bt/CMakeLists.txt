cmake_minimum_required(VERSION 3.8)
project(jev_ai_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(jev_ai_cpp REQUIRED)
find_package(rclcpp REQUIRED)

# Decide, JevAsk, Reason: link it and call jev_ai_bt::register_nodes(), or load it as a plugin
# (factory.registerFromPlugin(".../libjev_ai_bt.so"))
add_library(jev_ai_bt SHARED src/nodes.cpp)
target_compile_definitions(jev_ai_bt PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(jev_ai_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(jev_ai_bt behaviortree_cpp jev_ai_cpp rclcpp)

add_executable(bt_toy examples/bt_toy.cpp)
target_link_libraries(bt_toy jev_ai_bt)
ament_target_dependencies(bt_toy ament_index_cpp behaviortree_cpp rclcpp)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY bt DESTINATION share/${PROJECT_NAME})
install(TARGETS jev_ai_bt EXPORT export_jev_ai_bt
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS bt_toy RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_jev_ai_bt HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp jev_ai_cpp rclcpp)
ament_package()
