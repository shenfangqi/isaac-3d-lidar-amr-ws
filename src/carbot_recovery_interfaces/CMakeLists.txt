cmake_minimum_required(VERSION 3.8)
project(carbot_recovery_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RuntimeState.msg" "msg/ControllerFailure.msg" "msg/CostmapEvidence.msg"
  "msg/NearFieldEvidence.msg"
  "srv/RecoveryStep.srv" "action/BoundedRecovery.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs nav2_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
