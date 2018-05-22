cmake_minimum_required(VERSION 2.8.3)
project(sequoia_lane_detection)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  tf
  costmap_converter
)
find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detection_node
  src/lane_detection_node.cpp
  src/LaneDetection.cpp
)
target_link_libraries(lane_detection_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    lane_detection_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
