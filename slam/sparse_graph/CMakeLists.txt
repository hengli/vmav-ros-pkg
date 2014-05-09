cmake_minimum_required(VERSION 2.8.3)
project(sparse_graph)

find_package(catkin REQUIRED cauldron ceres cmake_modules roscpp sensor_msgs visualization_msgs)

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sparse_graph
  CATKIN_DEPENDS cauldron roscpp sensor_msgs visualization_msgs
  DEPENDS eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(sparse_graph
  src/Pose.cpp
  src/SparseGraph.cpp
  src/SparseGraphViz.cpp
  src/Transform.cpp
)

target_link_libraries(sparse_graph
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
