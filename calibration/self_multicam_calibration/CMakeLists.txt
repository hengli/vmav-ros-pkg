cmake_minimum_required(VERSION 2.8.3)
project(self_multicam_calibration)

find_package(catkin REQUIRED COMPONENTS
  camera_calibration
  cv_bridge
  hand_eye_calibration
  pose_graph
  pose_imu_calibration
  stereo_vo
)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_executable(self_multicam_calibration_node
  src/SelfMultiCamCalibration.cpp
  src/self_multicam_calibration_node.cpp
)

target_link_libraries(self_multicam_calibration_node
  ${catkin_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_executable(write_camera_info
  src/write_camera_info.cpp
)

target_link_libraries(write_camera_info
  ${catkin_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_executable(visualize_camera_system
  src/visualize_camera_system.cpp
)

target_link_libraries(visualize_camera_system
  ${catkin_LIBRARIES}
)
