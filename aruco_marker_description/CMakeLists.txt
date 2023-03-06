cmake_minimum_required(VERSION 3.0.0)
project(aruco_marker_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
