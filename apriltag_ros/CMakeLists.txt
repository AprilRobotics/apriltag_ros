cmake_minimum_required(VERSION 3.5)
project(atlas_apriltag_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lgtest -lgmock -g)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ageve_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(apriltag REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(image_geometry REQUIRED)



set(dependencies
  rclcpp
  rclcpp_components
  ageve_interfaces
  cv_bridge
  image_transport
  rcutils
  sensor_msgs
  tf2_ros
  tf2
  apriltag
  Eigen3
  OpenCV
  image_geometry
  yaml-cpp
  geometry_msgs
)


# Include directories
include_directories(
  include
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")  



# Add cpp libraries
add_library(atlas_apriltag_ros_component SHARED
  "src/continuous_detector.cpp"
  "src/common_functions.cpp"
)

target_link_libraries(atlas_apriltag_ros_component yaml-cpp apriltag)

target_compile_definitions(atlas_apriltag_ros_component
                           PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(atlas_apriltag_ros_component
                          rclcpp
                          rclcpp_components
                          ageve_interfaces
                          cv_bridge
                          image_transport
                          rcutils
                          sensor_msgs
                          tf2_ros
                          tf2
                          apriltag
                          Eigen3
                          OpenCV
                          image_geometry
                          yaml-cpp
                          geometry_msgs
                          )

rclcpp_components_register_node(atlas_apriltag_ros_component PLUGIN "atlas_apriltag_ros::ContinuousDetector" EXECUTABLE ContinuousDetector)
set(node_plugins "${node_plugins}atlas_apriltag_ros::ContinuousDetector;$<TARGET_FILE:atlas_apriltag_ros_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  atlas_apriltag_ros_component
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS atlas_apriltag_ros_component
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )


install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
) 

ament_package()