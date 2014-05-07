cmake_minimum_required(VERSION 2.8.3)
project(dutils)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/dutils
  LIBRARIES dutils
)

include_directories(include/dutils)

add_library(dutils
  src/BinaryFile.cpp
  src/ConfigFile.cpp
  src/DebugFunctions.cpp
  src/FileFunctions.cpp
  src/LineFile.cpp
  src/LUT.cpp
  src/Profiler.cpp
  src/Random.cpp
  src/StringFunctions.cpp
  src/TimeManager.cpp
  src/Timestamp.cpp
)

target_link_libraries(dutils
  ${catkin_LIBRARIES}
)
