cmake_minimum_required(VERSION 2.8.3)
project(sensor_models)

find_package(catkin REQUIRED cmake_modules tf)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_models
  CATKIN_DEPENDS tf
  DEPENDS eigen
)

###########
## Build ##
###########

include_directories(include ${Eigen_INCLUDE_DIRS})

add_library(sensor_models
  src/LaserSensorModel.cpp
  src/StereoSensorModel.cpp
)

add_executable(StereoSensorModel-sim src/stereo_sensor_model_sim.cpp)
target_link_libraries(StereoSensorModel-sim
  ${catkin_LIBRARIES}
  sensor_models
)
