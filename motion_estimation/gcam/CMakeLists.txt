cmake_minimum_required(VERSION 2.8.3)
project(gcam)

find_package(catkin REQUIRED COMPONENTS camera_systems ceres cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gcam
  CATKIN_DEPENDS camera_systems ceres
  DEPENDS eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(gcam
  src/GCamIMU.cpp
)

target_link_libraries(gcam
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

catkin_add_gtest(GCamIMU-test test/GCamIMU_test.cpp)
if(TARGET GCamIMU-test)
  target_link_libraries(GCamIMU-test gcam)
endif()
