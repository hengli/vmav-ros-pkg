cmake_minimum_required(VERSION 2.8.3)
project(pugixml)

find_package(catkin REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pugixml
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(pugixml
  src/pugixml.cpp
)

target_link_libraries(pugixml
  ${catkin_LIBRARIES}
)
