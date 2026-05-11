cmake_minimum_required(VERSION 3.8)
project(go2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/SemanticTask.msg"
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/InstanceMask.msg"
  "msg/InstanceMaskArray.msg"
  "msg/ObjectObservation.msg"
  "msg/ObjectObservationArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/SemanticEntity.msg"
  "msg/SemanticEntityArray.msg"
  "msg/SelectedTarget.msg"
)

set(srv_files
  "srv/GetFrontiers.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
