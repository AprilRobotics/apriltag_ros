cmake_minimum_required(VERSION 3.5)

project(apriltag_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "srv/AnalyzeSingleImage.srv"
  DEPENDENCIES
    "builtin_interfaces"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
