cmake_minimum_required(VERSION 2.8.3)
project(sequoia_gazebo)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  gazebo_ros
  dynamic_reconfigure
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/CameraControlPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    sequoia_interface_plugin
    camera_control_plugin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)

add_library(sequoia_interface_plugin
  src/SequoiaInterfacePlugin.cpp
)
target_link_libraries(sequoia_interface_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(camera_control_plugin
  src/CameraControlPlugin.cpp
)
add_dependencies(camera_control_plugin
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(camera_control_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Gazebo plugins
install(
  TARGETS
    sequoia_interface_plugin
    camera_control_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Config files
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
