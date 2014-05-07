cmake_minimum_required(VERSION 2.8.3)
project(location_recognition)

find_package(catkin REQUIRED COMPONENTS dbow2 dloopdetector sparse_graph)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES location_recognition
  CATKIN_DEPENDS dbow2 dloopdetector sparse_graph
  DEPENDS eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
  include
)

add_library(location_recognition
  src/OrbLocationRecognition.cpp
)

target_link_libraries(location_recognition
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(train_orb_location_recognition
  src/train_orb_location_recognition.cpp
)

target_link_libraries(train_orb_location_recognition
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  location_recognition
)