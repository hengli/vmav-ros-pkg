cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration)

find_package(catkin REQUIRED COMPONENTS camera_models camera_systems cauldron ceres cmake_modules cv_bridge image_transport px_comm roscpp sparse_graph)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_calibration
  CATKIN_DEPENDS camera_models camera_systems cauldron ceres px_comm roscpp sparse_graph
  DEPENDS boost eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(camera_calibration
  src/CameraCalibration.cpp
  src/Chessboard.cpp
  src/StereoCameraCalibration.cpp
)

target_link_libraries(camera_calibration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(mono_camera_calibration
  src/mono_camera_calibration.cpp
)

target_link_libraries(mono_camera_calibration
  camera_calibration
)

add_executable(stereo_camera_calibration
  src/stereo_camera_calibration.cpp
)

target_link_libraries(stereo_camera_calibration
  camera_calibration
)

add_executable(convert_stereo_calibration_data
  src/convert_stereo_calibration_data.cpp
)

target_link_libraries(convert_stereo_calibration_data
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  camera_calibration
)
