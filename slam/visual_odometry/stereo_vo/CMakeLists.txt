cmake_minimum_required(VERSION 2.8.3)
project(stereo_vo)

find_package(catkin REQUIRED COMPONENTS camera_systems cauldron ceres cmake_modules cv_bridge image_transport pose_estimation px_comm roscpp sparse_graph)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stereo_vo
  CATKIN_DEPENDS camera_systems cauldron ceres pose_estimation sparse_graph
  DEPENDS eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(stereo_vo
  src/LocalStereoBA.cpp
  src/StereoVO.cpp
)

add_dependencies(stereo_vo px_comm_gencpp)

target_link_libraries(stereo_vo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(stereo_vo_node
  src/stereo_vo_node.cpp
)

target_link_libraries(stereo_vo_node
  ${catkin_LIBRARIES}
  stereo_vo
)
