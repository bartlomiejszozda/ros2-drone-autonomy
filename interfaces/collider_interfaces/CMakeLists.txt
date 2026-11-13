cmake_minimum_required(VERSION 3.8)
project(collider_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PrepareDrone.action"
  "action/Takeoff.action"
  "action/Mission.action"
  "action/Ask.action"
  "msg/TargetObservation.msg"
  "msg/TargetTrack.msg"
  "msg/CarDetection.msg"
  "msg/CarDetections.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
