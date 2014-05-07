cmake_minimum_required(VERSION 2.8.3)
project(pose_imu_calibration)

find_package(catkin REQUIRED cauldron ceres geometry_msgs rosbag sensor_msgs sparse_graph)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_imu_calibration
  CATKIN_DEPENDS cauldron ceres sensor_msgs sparse_graph
  DEPENDS boost
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

add_library(pose_imu_calibration
  src/PoseIMUCalibration.cpp
)

target_link_libraries(pose_imu_calibration
  ${catkin_LIBRARIES}
)

add_executable(vicon_imu_calibration
  src/vicon_imu_calibration.cpp
)

target_link_libraries(vicon_imu_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  pose_imu_calibration
)

#############
## Testing ##
#############

catkin_add_gtest(PoseIMUCalibration-test test/PoseIMUCalibration_test.cc)
if(TARGET PoseIMUCalibration-test)
  target_link_libraries(PoseIMUCalibration-test pose_imu_calibration)
endif()
