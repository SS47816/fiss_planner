cmake_minimum_required(VERSION 3.16)
project(frenet_planner_superbuild LANGUAGES CXX)

# Top-level entry point for standalone (non-ROS) builds and for the Python
# package. ROS builds use core/, msgs/ and ros/* directly as packages.
option(FRENET_PLANNER_BUILD_PYTHON "Build the Python bindings" OFF)

enable_testing()
add_subdirectory(core)

if(FRENET_PLANNER_BUILD_PYTHON)
  add_subdirectory(python)
endif()
