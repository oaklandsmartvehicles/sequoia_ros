cmake_minimum_required(VERSION 2.8.3)
project(stop_sign_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  sign_detection	
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(stop_sign_plan
  src/stop_sign_plan.cpp
)
add_dependencies(stop_sign_plan
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(stop_sign_plan
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    stop_sign_plan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
