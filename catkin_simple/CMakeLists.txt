cmake_minimum_required(VERSION 2.8.3)
project(catkin_simple)

set(OMPL_DIR /opt/ros/noetic/include/ompl)

find_package(catkin REQUIRED)
find_package(ompl REQUIRED)


catkin_package(
  CATKIN_DEPENDS catkin
  CFG_EXTRAS catkin_simple-extras.cmake
)
