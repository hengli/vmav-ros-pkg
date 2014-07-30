cmake_minimum_required(VERSION 2.8.3)
project(rviz_dynocmap_plugin)

find_package(catkin REQUIRED COMPONENTS
  cauldron
  dynocmap
  rviz
)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_FILES
  include/rviz_dynocmap_plugin/dynocmap_display.h
)

catkin_package(
  CATKIN_DEPENDS rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(rviz_dynocmap_plugin
  src/dynocmap_display.cpp
  src/dynocmap_visual.cpp
  ${MOC_FILES}
)

add_dependencies(rviz_dynocmap_plugin rviz_dynocmap_msgs_generate_messages_cpp)

target_link_libraries(rviz_dynocmap_plugin
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
