cmake_minimum_required(VERSION 2.8.3)
project(apriltags2)

set(CMAKE_C_FLAGS "-std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -I. -O4 -fno-strict-overflow")

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltags2
  CATKIN_DEPENDS
)

include_directories(include include/common)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(apriltags2
  src/apriltag.c
  src/apriltag_quad_thresh.c
  src/g2d.c
  src/getopt.c
  src/homography.c
  src/image_f32.c
  src/image_u8.c
  src/image_u8x3.c
  src/image_u8x4.c
  src/matd.c
  src/pam.c
  src/pjpeg.c
  src/pjpeg-idct.c
  src/pnm.c
  src/string_util.c
  src/svd22.c
  src/tag16h5.c
  src/tag25h7.c
  src/tag25h9.c
  src/tag36artoolkit.c
  src/tag36h10.c
  src/tag36h11.c
  src/time_util.c
  src/unionfind.c
  src/workerpool.c
  src/zarray.c
  src/zhash.c
  src/zmaxheap.c)
target_link_libraries(apriltags2
 ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS apriltags2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
