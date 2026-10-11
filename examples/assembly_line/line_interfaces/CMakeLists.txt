cmake_minimum_required(VERSION 3.8)
project(line_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/PartState.msg"
  "msg/LineState.msg"
  "msg/QaReading.msg"
  "msg/PartOutcome.msg"
  "srv/Release.srv"
  "srv/Route.srv"
  "srv/Dispose.srv"
  "action/Operate.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
