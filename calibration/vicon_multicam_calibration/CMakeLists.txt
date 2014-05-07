cmake_minimum_required(VERSION 2.8.3)
project(vicon_multicam_calibration)

find_package(catkin REQUIRED
  camera_calibration
  cv_bridge
  image_transport
  pose_imu_calibration
)

find_package(Boost REQUIRED COMPONENTS program_options thread)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_executable(vicon_multicam_calibration_node
  src/ViconMultiCamCalibration.cpp
  src/vicon_multicam_calibration_node.cpp
)

target_link_libraries(vicon_multicam_calibration_node
  ${catkin_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_executable(diff_vicon_multicam_calibration
  src/diff_vicon_multicam_calibration.cpp
)

target_link_libraries(diff_vicon_multicam_calibration
  ${catkin_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
