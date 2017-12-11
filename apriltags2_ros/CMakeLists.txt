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

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

add_definitions (-Wall -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros ")

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  message("enabling msse2 for x86_64 or x86_32 architecture")
  #  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse2 ")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)
  message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)

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
