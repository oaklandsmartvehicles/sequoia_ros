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

install(
  TARGETS
    task_q2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
