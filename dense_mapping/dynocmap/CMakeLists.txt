cmake_minimum_required(VERSION 2.8.3)
project(dynocmap)

find_package(catkin REQUIRED COMPONENTS cmake_modules dynocmap_msgs eigen_conversions pcl_ros sensor_models)

find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynocmap
  CATKIN_DEPENDS dynocmap_msgs pcl_ros sensor_models
  DEPENDS eigen opencv
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(dynocmap
  src/OccupancyTile.cpp
  src/DynocMap.cpp
  src/OcNode.cpp
  src/OcTree.cpp
  src/OcTreeCache.cpp
)

add_dependencies(dynocmap
  dynanocmap_msgs_generate_messages_cpp
)

target_link_libraries(dynocmap
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Testing ##
#############

catkin_add_gtest(OcTree-test test/OcTree_test.cpp)
if(TARGET OcTree-test)
  target_link_libraries(OcTree-test dynocmap)
endif()

catkin_add_gtest(DynocMap-test test/DynocMap_test.cpp)
if(TARGET DynocMap-test)
  target_link_libraries(DynocMap-test dynocmap)
endif()
