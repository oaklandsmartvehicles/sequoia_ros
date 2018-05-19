cmake_minimum_required(VERSION 2.8.3)
project(sequoia_twist_controller)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/SequoiaTwist.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/sequoia_twist_control_node.cpp
  src/SequoiaTwistController.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(drive_control_msg_conversion
  src/drive_control_msg_conversion.cpp
)
target_link_libraries(drive_control_msg_conversion
  ${catkin_LIBRARIES}
)

### Install Rules
install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
