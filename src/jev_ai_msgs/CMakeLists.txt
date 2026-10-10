cmake_minimum_required(VERSION 3.8)
project(jev_ai_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Question.msg"
  "msg/Distribution.msg"
  "msg/ClassRef.msg"
  "msg/Decision.msg"
  "msg/QuestionTrace.msg"
  "msg/DecisionRecord.msg"
  "msg/Report.msg"
  "msg/Directive.msg"
  "msg/Label.msg"
  "srv/JevDecide.srv"
  "action/Reason.action"
  "action/Decide.action"
  DEPENDENCIES builtin_interfaces diagnostic_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
