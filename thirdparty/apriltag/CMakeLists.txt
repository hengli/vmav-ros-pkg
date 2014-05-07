cmake_minimum_required(VERSION 2.8.3)
project(apriltag)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltag
)

###########
## Build ##
###########

set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-std=gnu99")

include_directories(
  include
  include/apriltag
)

add_library(apriltag
  src/apriltag.c
  src/tagtest.c
  src/image_f32.c
  src/image_u8.c
  src/image_u32.c
  src/unionfind.c
  src/zhash.c
  src/zarray.c
  src/matd.c
  src/homography.c
  src/graymodel.c
  src/tag36h11.c
  src/tag36h10.c
  src/segment2.c
  src/workerpool.c
  src/g2d.c
)

target_link_libraries(apriltag
  ${catkin_LIBRARIES}
)
