cmake_minimum_required(VERSION 2.8.3)
project(task_nodes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lane_keep_and_stop
  src/lane_keep_and_stop.cpp
)
target_link_libraries(lane_keep_and_stop
  ${catkin_LIBRARIES}
)

add_executable(turn_and_stop
  src/turn_and_stop.cpp
)
target_link_libraries(turn_and_stop
  ${catkin_LIBRARIES}
)

add_executable(lane_keep_and_stop1
  src/lane_keep_and_stop1.cpp
)
target_link_libraries(lane_keep_and_stop1
  ${catkin_LIBRARIES}
)


add_executable(multiplexor
  src/multiplexor.cpp
)
target_link_libraries(multiplexor
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    multiplexor
    lane_keep_and_stop
    turn_and_stop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
