cmake_minimum_required(VERSION 2.8.3)
project(mav_path_smoothing)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/loco_smoother_mgv.cpp
  src/path_smoother_base_mgv.cpp
  src/polynomial_smoother_mgv.cpp
  src/velocity_ramp_smoother.cpp
)
target_link_libraries(${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
