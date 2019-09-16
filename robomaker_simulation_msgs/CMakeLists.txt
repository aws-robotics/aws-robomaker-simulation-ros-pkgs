cmake_minimum_required(VERSION 3.7.2)
project(robomaker_simulation_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Tag.msg"
)
set(srv_files
  "srv/AddTags.srv"
  "srv/Cancel.srv"
  "srv/ListTags.srv"
  "srv/RemoveTags.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
