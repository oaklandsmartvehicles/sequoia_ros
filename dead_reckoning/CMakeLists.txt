cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(dead_reckoning
  src/dead_reckoning_node.cpp
  src/DeadReckoning.cpp
)
target_link_libraries(dead_reckoning ${catkin_LIBRARIES})

install(
  TARGETS
    dead_reckoning
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
