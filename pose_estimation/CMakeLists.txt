cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)

find_package(catkin REQUIRED camera_systems cauldron ceres)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_estimation
  CATKIN_DEPENDS cauldron
  DEPENDS eigen
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

add_library(pose_estimation
  src/gP3P.cpp
  src/P3P.cpp
)

link_libraries(pose_estimation
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

catkin_add_gtest(pose_estimation-test test/pose_estimation_test.cpp)
if(TARGET pose_estimation-test)
  target_link_libraries(pose_estimation-test pose_estimation)
endif()
