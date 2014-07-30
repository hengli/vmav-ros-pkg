cmake_minimum_required(VERSION 2.8.3)
project(dynocmap_mapping)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynocmap
  message_filters
  pcl_conversions
  roscpp
  sensor_msgs
  tf_conversions
)

find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynocmap_mapping
#  CATKIN_DEPENDS dynocmap roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

## Declare a cpp library
# add_library(dynocmap_mapping
#   src/${PROJECT_NAME}/dynocmap_mapping.cpp
# )

add_executable(dynocmap_mapping_sim_node
  src/dynocmap_mapping_sim_node.cpp
)

add_dependencies(dynocmap_mapping_sim_node dynocmap_msgs_generate_messages_cpp)

target_link_libraries(dynocmap_mapping_sim_node
  ${catkin_LIBRARIES}
)
