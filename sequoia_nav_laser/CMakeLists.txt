cmake_minimum_required(VERSION 2.8.3)
project(sequoia_nav_laser)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
