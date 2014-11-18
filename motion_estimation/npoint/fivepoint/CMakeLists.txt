cmake_minimum_required(VERSION 2.8.3)
project(fivepoint)

find_package(catkin REQUIRED)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fivepoint
  DEPENDS opencv
)

include_directories(include)

add_library(fivepoint
  src/fivepoint.cpp
  src/modelest.cpp
)

target_link_libraries(fivepoint
  ${catkin_LIBRARIES}
)
