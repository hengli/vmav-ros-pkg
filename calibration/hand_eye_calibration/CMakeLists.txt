cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibration)

find_package(catkin REQUIRED COMPONENTS cauldron ceres cmake_modules)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hand_eye_calibration
  CATKIN_DEPENDS cauldron ceres
  DEPENDS eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

add_library(hand_eye_calibration
  src/HandEyeCalibration.cpp
)

target_link_libraries(hand_eye_calibration
  ${catkin_LIBRARIES}
)

catkin_add_gtest(HandEyeCalibration-test test/HandEyeCalibration_test.cpp)
if(TARGET HandEyeCalibration-test)
  target_link_libraries(HandEyeCalibration-test hand_eye_calibration)
endif()
