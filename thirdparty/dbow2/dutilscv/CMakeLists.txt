cmake_minimum_required(VERSION 2.8.3)
project(dutilscv)

find_package(catkin REQUIRED)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/dutilscv
  LIBRARIES dutilscv
  DEPENDS opencv
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/dutilscv
)

add_library(dutilscv
  src/Drawing.cpp
  src/Geometry.cpp
  src/GUI.cpp
  src/IO.cpp
  src/Mat.cpp
  src/Transformations.cpp
  src/Types.cpp
)

target_link_libraries(dutilscv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
