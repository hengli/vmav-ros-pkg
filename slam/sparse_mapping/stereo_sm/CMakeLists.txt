cmake_minimum_required(VERSION 2.8.3)
project(stereo_sm)

find_package(catkin REQUIRED COMPONENTS
  ceres
  cv_bridge
  pose_graph
  px_comm
  rosbag
  roscpp
  stereo_vo
)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_executable(stereo_sm_node
  src/StereoSM.cpp
  src/stereo_sm_node.cpp
)

target_link_libraries(stereo_sm_node
  ${catkin_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
