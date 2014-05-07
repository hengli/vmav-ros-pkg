cmake_minimum_required(VERSION 2.8.3)
project(dvision)

find_package(catkin REQUIRED COMPONENTS dutils dutilscv)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/dvision
  LIBRARIES dvision
  CATKIN_DEPENDS dutils dutilscv
  DEPENDS opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/dvision
)

add_library(dvision
  src/BRIEF.cpp
  src/BundleCamera.cpp
  src/FSolver.cpp
  src/HSolver.cpp
  src/ImageFunctions.cpp
  src/Matches.cpp
  src/ORB.cpp
  src/PatchFile.cpp
  src/PixelPointFile.cpp
  src/PLYFile.cpp
  src/PMVSCamera.cpp
  src/SurfSet.cpp
)

target_link_libraries(dvision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
