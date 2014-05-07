cmake_minimum_required(VERSION 2.8.3)
project(vicon_client)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure geometry_msgs roscpp)

generate_dynamic_reconfigure_options(
  cfg/ViconClient.cfg
)

if(APPLE)

else(APPLE)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  link_directories(lib/x86_64)
endif()

add_library(vicon_client
  src/ViconClient.cpp
)

target_link_libraries(vicon_client
  ViconDataStreamSDK_CPP
)

add_executable(vicon_client_node
  src/vicon_client_node.cpp
)  

add_dependencies(vicon_client_node ${PROJECT_NAME}_gencfg)

target_link_libraries(vicon_client_node
  ${catkin_LIBRARIES}
  vicon_client
)

endif(APPLE)
