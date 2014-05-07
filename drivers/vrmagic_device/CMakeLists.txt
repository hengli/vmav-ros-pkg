cmake_minimum_required(VERSION 2.8.3)
project(vrmagic_device)

find_package(catkin REQUIRED COMPONENTS asctec_hl_comm camera_info_manager dds_ros driver_base dynamic_reconfigure image_transport nodelet px_comm tf)

generate_dynamic_reconfigure_options(
  cfg/VRmagicDevice.cfg
)

# Make CMake aware of the cmake folder for local FindXXX scripts.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(RTI REQUIRED)

catkin_package(
  CATKIN_DEPENDS asctec_hl_comm camera_info_manager dds_ros image_transport nodelet
)

###########
## Build ##
###########

#if(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=cortex-a8 -march=armv7-a -mfloat-abi=hard -mfpu=neon")
#endif()

set(NDDSHOME $ENV{NDDSHOME})
set(syslibs "-ldl -lnsl -lm -lpthread -lrt")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${NDDSHOME}/include
  ${NDDSHOME}/include/ndds
)

add_definitions(-DRTI_UNIX -DRTI_LINUX)

link_directories(
  ${NDDSHOME}/lib/x64Linux2.6gcc4.4.5
  ${NDDSHOME}/lib/armv7neonhfLinux3.xgcc4.6.3
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  link_directories(lib/x86_64)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
  link_directories(lib/arm)
endif()

add_library(vrmagic_device_driver
  src/VRmagicCamera.cpp
  src/VRmagicDeviceDriver.cpp
  src/vrmusbcamcpp.cpp
)  

add_dependencies(vrmagic_device_driver ${PROJECT_NAME}_gencfg asctec_hl_comm_gencpp px_comm_gencpp) 

target_link_libraries(vrmagic_device_driver
  ${catkin_LIBRARIES}
  ${syslibs}
  nddscz
  nddscppz
  nddscorez
  vrmusbcam2
)  

add_executable(vrmagic_device_node
  src/vrmagic_device_node.cpp
)

target_link_libraries(vrmagic_device_node
  ${catkin_LIBRARIES}
  vrmagic_device_driver
)

add_library(vrmagic_device_nodelet
  src/vrmagic_device_nodelet.cpp
)

target_link_libraries(vrmagic_device_nodelet
  ${catkin_LIBRARIES}
  vrmagic_device_driver
)
