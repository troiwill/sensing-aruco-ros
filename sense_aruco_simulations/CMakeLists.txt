cmake_minimum_required(VERSION 3.0.0)
project(sense_aruco_simulations)

find_package(catkin REQUIRED
)

catkin_package()

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
