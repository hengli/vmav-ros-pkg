cmake_minimum_required(VERSION 2.8.3)
project(gcam_vo)

find_package(catkin REQUIRED COMPONENTS
  camera_systems
  ceres
  cv_bridge
  gcam
  message_filters
  pose_estimation
  px_comm
  roscpp
  sparse_graph
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gcam_vo
  CATKIN_DEPENDS camera_systems cauldron ceres pose_estimation sparse_graph
  DEPENDS eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(gcam_vo
  src/GCamLocalBA.cpp
  src/GCamVO.cpp
)

add_dependencies(gcam_vo px_comm_gencpp)

target_link_libraries(gcam_vo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(gcam_vo_node
  src/gcam_vo_node.cpp
)

target_link_libraries(gcam_vo_node
  ${catkin_LIBRARIES}
  gcam_vo
)
