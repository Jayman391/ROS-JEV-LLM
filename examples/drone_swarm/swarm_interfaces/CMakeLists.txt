cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneStatus.msg"
  "msg/GroundTruth.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/LinkStatus.msg"
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "srv/Allocate.srv"
  "action/Task.action"
  "action/Mission.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
