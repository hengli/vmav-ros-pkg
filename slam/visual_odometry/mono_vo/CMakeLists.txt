cmake_minimum_required(VERSION 2.8.3)
project(mono_vo)

find_package(catkin REQUIRED COMPONENTS camera_systems cauldron ceres cmake_modules cv_bridge fivepoint image_transport pose_estimation px_comm roscpp sparse_graph)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mono_vo
  CATKIN_DEPENDS camera_systems cauldron ceres fivepoint pose_estimation sparse_graph
  DEPENDS eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(mono_vo
  src/LocalMonoBA.cpp
  src/MonoVO.cpp
)

add_dependencies(mono_vo px_comm_gencpp)

target_link_libraries(mono_vo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(mono_vo_node
  src/mono_vo_node.cpp
)

target_link_libraries(mono_vo_node
  ${catkin_LIBRARIES}
  mono_vo
)
