cmake_minimum_required(VERSION 2.8.3)
project(camera_models)

find_package(catkin REQUIRED COMPONENTS cauldron ceres cmake_modules px_comm)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_models
 CATKIN_DEPENDS cauldron ceres px_comm
 DEPENDS eigen opencv
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(camera_models
  src/Camera.cpp
  src/CameraFactory.cpp
  src/CataCamera.cpp
  src/CostFunctionFactory.cpp
  src/EquidistantCamera.cpp
  src/PinholeCamera.cpp
)

add_dependencies(camera_models px_comm_gencpp)

target_link_libraries(camera_models
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Testing ##
#############

catkin_add_gtest(CataCamera-test test/CataCamera_test.cpp)
if(TARGET CataCamera-test)
  target_link_libraries(CataCamera-test camera_models)
endif()

catkin_add_gtest(EquidistantCamera-test test/EquidistantCamera_test.cpp)
if(TARGET EquidistantCamera-test)
  target_link_libraries(EquidistantCamera-test camera_models)
endif()

catkin_add_gtest(PinholeCamera-test test/PinholeCamera_test.cpp)
if(TARGET PinholeCamera-test)
  target_link_libraries(PinholeCamera-test camera_models)
endif()
