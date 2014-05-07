cmake_minimum_required(VERSION 2.8.3)
project(dbow2)

find_package(catkin REQUIRED COMPONENTS dutils dutilscv dvision)

catkin_package(
  INCLUDE_DIRS include include/dbow2
  LIBRARIES dbow2
  CATKIN_DEPENDS dutils dutilscv dvision
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/dbow2
)

add_library(dbow2
  src/BowVector.cpp
  src/FBrief.cpp
  src/FeatureVector.cpp
  src/FOrb.cpp
  src/FSurf64.cpp
  src/QueryResults.cpp
  src/ScoringObject.cpp
)

target_link_libraries(dbow2
  ${catkin_LIBRARIES}
)

