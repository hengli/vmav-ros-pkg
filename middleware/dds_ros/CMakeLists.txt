cmake_minimum_required(VERSION 2.8.3)
project(dds_ros)

find_package(catkin REQUIRED COMPONENTS image_transport nodelet px_comm)

# Make CMake aware of the cmake folder for local FindXXX scripts.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(RTI REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dds_ros_utils
  CATKIN_DEPENDS image_transport nodelet px_comm
  DEPENDS nddsc nddscpp nddscore
)

set(NDDSHOME $ENV{NDDSHOME})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${NDDSHOME}/include
  ${NDDSHOME}/include/ndds
  include
)

add_definitions(-DRTI_UNIX)

link_directories(
  ${NDDSHOME}/lib/x64Linux2.6gcc4.4.5
  ${NDDSHOME}/lib/armv7neonhfLinux3.xgcc4.6.3
)

add_library(dds_ros_utils
  src/utils.cpp
)

target_link_libraries(dds_ros_utils
  ${catkin_LIBRARIES}
  nddsc
  nddscpp
  nddscore
)

add_library(dds_to_ros_bridge
  src/DdsRosBridge.cpp
)

target_link_libraries(dds_to_ros_bridge
  ${catkin_LIBRARIES}
  dds_ros_utils
  nddsc
  nddscpp
  nddscore
)

add_library(ros_to_dds_bridge
  src/RosDdsBridge.cpp
)

target_link_libraries(ros_to_dds_bridge
  ${catkin_LIBRARIES}
  dds_ros_utils
  nddsc
  nddscpp
  nddscore
)

add_executable(dds_to_ros_bridge_node
  src/dds_to_ros_bridge_node.cpp
)

target_link_libraries(dds_to_ros_bridge_node
  ${catkin_LIBRARIES}
  dds_to_ros_bridge
)

add_executable(ros_to_dds_bridge_node
  src/ros_to_dds_bridge_node.cpp
)

target_link_libraries(ros_to_dds_bridge_node
  ${catkin_LIBRARIES}
  ros_to_dds_bridge
)

add_library(dds_to_ros_bridge_nodelet
  src/dds_to_ros_bridge_nodelet.cpp
)

target_link_libraries(dds_to_ros_bridge_nodelet
  ${catkin_LIBRARIES}
  dds_to_ros_bridge
)
