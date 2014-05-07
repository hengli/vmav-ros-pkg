cmake_minimum_required(VERSION 2.8.3)
project(dloopdetector)

find_package(catkin REQUIRED COMPONENTS dbow2)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dbow2
)
