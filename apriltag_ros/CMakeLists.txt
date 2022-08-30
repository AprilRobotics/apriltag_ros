cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# We default to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_compile_options(
  "-O3" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno"
  "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")

# Note: These options have been turned off to allow for binary releases -
# in local builds, they can be reactivated to achieve higher performance.
# if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64" OR "x86_32")
#   message("enabling msse2 for x86_64 or x86_32 architecture")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse2 ")
# endif()
# if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
#   message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
#   set(CMAKE_CXX_FLAGS
#     "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
# endif()


find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "srv/AnalyzeSingleImage.srv"
  DEPENDENCIES
    geometry_msgs sensor_msgs std_msgs
)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_common SHARED
  src/common_functions.cpp
)
target_link_libraries(${PROJECT_NAME}_common
  ${OpenCV_LIBRARIES}
  apriltag::apriltag
)
if(${tf2_geometry_msgs_VERSION} VERSION_GREATER_EQUAL 0.18.0)
  target_compile_definitions(${PROJECT_NAME}_common PRIVATE
    USE_TF2_GEOMETRY_MSGS_HPP_HEADER=1
  )
endif()

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${PROJECT_NAME}_common
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME}_common "${cpp_typesupport_target}")
endif()

ament_auto_add_library(${PROJECT_NAME}_continuous_detector SHARED
  src/continuous_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_continuous_detector
  ${PROJECT_NAME}_common
)
rclcpp_components_register_node(${PROJECT_NAME}_continuous_detector
  PLUGIN "apriltag_ros::ContinuousDetector"
  EXECUTABLE ${PROJECT_NAME}_continuous_detector_node
)

ament_auto_add_library(${PROJECT_NAME}_single_image_detector SHARED
  src/single_image_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_single_image_detector
  ${PROJECT_NAME}_common
)
rclcpp_components_register_node(${PROJECT_NAME}_single_image_detector
  PLUGIN "apriltag_ros::SingleImageDetector"
  EXECUTABLE ${PROJECT_NAME}_single_image_detector_node
)

ament_auto_add_library(${PROJECT_NAME}_single_image_client SHARED
  src/single_image_client.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}_single_image_client
  PLUGIN "apriltag_ros::SingleImageClient"
  EXECUTABLE ${PROJECT_NAME}_single_image_client_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    scripts
)
