cmake_minimum_required(VERSION 2.8.3)
project(task_nodes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(task_q2
  src/task_q2.cpp
)
target_link_libraries(task_q2
  ${catkin_LIBRARIES}
)

add_executable(task_q5
  src/task_q5.cpp
)
target_link_libraries(task_q5
  ${catkin_LIBRARIES}
)

add_executable(stop_line_test
  src/stop_line_test.cpp
)
target_link_libraries(stop_line_test
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
    task_q2
    task_q5
    multiplexor
    stop_line_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
