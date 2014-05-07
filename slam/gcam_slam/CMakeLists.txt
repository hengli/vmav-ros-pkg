cmake_minimum_required(VERSION 2.8.3)
project(gcam_slam)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gcam_vo
  location_recognition
  message_filters
  px_comm
  roscpp
)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gcam_slam
  CATKIN_DEPENDS gcam_vo
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(gcam_slam
  src/GCamDWBA.cpp
  src/GCamSLAM.cpp
)

target_link_libraries(gcam_slam
  ${catkin_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_executable(gcam_slam_node
  src/gcam_slam_node.cpp
)

add_dependencies(gcam_slam_node px_comm_gencpp)

target_link_libraries(gcam_slam_node
  ${catkin_LIBRARIES}
  gcam_slam
)
