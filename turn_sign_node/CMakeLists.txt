cmake_minimum_required(VERSION 2.8.3)
project(turn_sign_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  sign_detection
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(turn_sign_plan
  src/turn_sign_plan.cpp
)
add_dependencies(turn_sign_plan
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(turn_sign_plan
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    turn_sign_plan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
