cmake_minimum_required(VERSION 2.8.3)
project(camera_systems)

find_package(catkin REQUIRED COMPONENTS camera_models geometry_msgs pugixml)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_systems
  CATKIN_DEPENDS camera_models geometry_msgs pugixml
  DEPENDS eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(camera_systems
  src/CameraSystem.cpp
)

target_link_libraries(camera_systems
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
