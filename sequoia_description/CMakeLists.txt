cmake_minimum_required(VERSION 2.8.3)
project(sequoia_description)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED)

catkin_package()

### Install Rules
install(
  DIRECTORY
    meshes
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN *.tar.gz EXCLUDE
)
