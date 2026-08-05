cmake_minimum_required(VERSION 3.8)
project(flir_ptz_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PtzState.msg"
  "msg/MoveToCmd.msg"
  "msg/JoyStickControlCmd.msg"
  "msg/ScanCmd.msg"
  "msg/ControlSource.msg"
  "srv/ClaimControl.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
