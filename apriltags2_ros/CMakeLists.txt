cmake_minimum_required(VERSION 2.8.3)
project(apriltags2_ros)

find_package(catkin REQUIRED COMPONENTS
  apriltags2
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  tf
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  AprilTagDetectionArray.msg
  AprilTagDetection.msg
)

add_service_files(
  FILES
  AnalyzeSingleImage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS apriltags2 geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime cv_bridge tf
  DEPENDS Eigen OpenCV
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(common src/common_functions.cpp)
add_dependencies(common ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(continuous_detector src/continuous_detector.cpp)
target_link_libraries(continuous_detector
  common
  ${catkin_LIBRARIES}
)

add_library(single_image_detector src/single_image_detector.cpp)
target_link_libraries(single_image_detector
  common
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_continuous_node src/${PROJECT_NAME}_continuous_node.cpp)
add_dependencies(${PROJECT_NAME}_continuous_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_continuous_node
  continuous_detector
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_single_image_server_node src/${PROJECT_NAME}_single_image_server_node.cpp)
add_dependencies(${PROJECT_NAME}_single_image_server_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_single_image_server_node
  single_image_detector
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_single_image_client_node src/${PROJECT_NAME}_single_image_client_node.cpp)
add_dependencies(${PROJECT_NAME}_single_image_client_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_single_image_client_node
  common
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# TODO uncomment these, such that can build as Release (faster)
#
### Mark executables and/or libraries for installation
#install(TARGETS apriltag2_detector apriltag2_detector_nodelet apriltag2_detector_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#
### Mark cpp header files for installation
#install(DIRECTORY include/apriltags2_ros
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  )
#
### Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES nodelet_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
