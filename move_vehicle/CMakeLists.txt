cmake_minimum_required(VERSION 2.8.3)
project(move_vehicle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure	
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(move_vehicle
  src/move_vehicle.cpp
)
add_dependencies(move_vehicle
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(move_vehicle
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  move_vehicle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
