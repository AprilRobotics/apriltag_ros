cmake_minimum_required(VERSION 3.5)
project(apriltag_ros)

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
find_package(apriltag_ros_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
  apriltag_ros_interfaces
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
  std_msgs
)


# Include directories
include_directories(
  include
)

# Add cpp libraries
add_library(${PROJECT_NAME} SHARED
  "src/continuous_detector.cpp"
  "src/single_image_detector.cpp"
  "src/common_functions.cpp"
)

target_link_libraries(${PROJECT_NAME} yaml-cpp apriltag)

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          rclcpp_components
                          apriltag_ros_interfaces
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
                          std_msgs
)

# Register nodes
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "apriltag_ros::ContinuousDetector" EXECUTABLE ContinuousDetector)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "apriltag_ros::SingleImageDetector" EXECUTABLE SingleImageDetector)

# Build executables
add_executable(single_image_client src/apriltag_ros_single_image_client_node.cpp) 
target_link_libraries(single_image_client ${PROJECT_NAME})

add_executable(single_image_server src/apriltag_ros_single_image_server_node.cpp) 
target_link_libraries(single_image_server ${PROJECT_NAME})

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  single_image_client
  single_image_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
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