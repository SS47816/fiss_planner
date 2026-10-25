cmake_minimum_required(VERSION 3.16)
project(fiss_planner_ros1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs visualization_msgs ackermann_msgs tf2 tf2_ros tf2_geometry_msgs
  dynamic_reconfigure fiss_planner_msgs
  lanelet2_core lanelet2_io lanelet2_projection lanelet2_routing lanelet2_traffic_rules)
find_package(fiss_planner REQUIRED)

generate_dynamic_reconfigure_options(cfg/FissPlanner.cfg)
catkin_package(CATKIN_DEPENDS roscpp nav_msgs geometry_msgs fiss_planner_msgs dynamic_reconfigure)

set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
include_directories(${COMMON_DIR}/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_lanelet2 ${COMMON_DIR}/src/lanelet2_adapter.cpp)
target_link_libraries(${PROJECT_NAME}_lanelet2 fiss_planner::fiss_planner ${catkin_LIBRARIES})

add_executable(planner_node src/planner_node.cpp)
add_dependencies(planner_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_node ${PROJECT_NAME}_lanelet2 fiss_planner::fiss_planner ${catkin_LIBRARIES})

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node fiss_planner::fiss_planner ${catkin_LIBRARIES})

install(TARGETS planner_node controller_node ${PROJECT_NAME}_lanelet2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_planner_node_ros1 test/planner_node.test test/test_planner_node.cpp)
  target_link_libraries(test_planner_node_ros1 ${catkin_LIBRARIES})
  add_dependencies(test_planner_node_ros1 planner_node ${catkin_EXPORTED_TARGETS})
endif()
