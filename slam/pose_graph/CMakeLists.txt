cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)

find_package(catkin REQUIRED COMPONENTS
  camera_systems
  cmake_modules
  location_recognition
  pose_estimation
  roscpp
  sparse_graph
  visualization_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_graph
  CATKIN_DEPENDS camera_systems location_recognition pose_estimation roscpp sparse_graph visualization_msgs
  DEPENDS eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

add_library(pose_graph
  src/PoseGraph.cpp
  src/PoseGraphViz.cpp
)

target_link_libraries(pose_graph
  ${catkin_LIBRARIES}
)
