cmake_minimum_required(VERSION 3.8)
project(unav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NedWaypoint.msg"
  "msg/NedPath.msg"
  "msg/MPCCommand.msg"
  "msg/ObjectGps.msg"
  "srv/UploadMission.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
