cmake_minimum_required(VERSION 2.8.3)
project(cauldron)

find_package(catkin REQUIRED COMPONENTS ceres cmake_modules)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cauldron
  DEPENDS eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(cauldron
  src/cauldron.cpp
  src/EigenQuaternionParameterization.cpp
  src/PLine.cpp
  src/PLineCorrespondence.cpp
)

target_link_libraries(cauldron
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
