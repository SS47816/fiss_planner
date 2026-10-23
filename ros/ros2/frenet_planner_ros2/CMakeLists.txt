cmake_minimum_required(VERSION 3.16)
project(frenet_planner_ros2 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(frenet_planner REQUIRED)
find_package(frenet_planner_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)

add_library(${PROJECT_NAME} SHARED
  src/planner_node.cpp
  src/controller_node.cpp
  ${COMMON_DIR}/src/lanelet2_adapter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${COMMON_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} frenet_planner::frenet_planner)
ament_target_dependencies(${PROJECT_NAME}
  frenet_planner_msgs rclcpp rclcpp_components nav_msgs geometry_msgs visualization_msgs ackermann_msgs
  tf2 tf2_ros tf2_geometry_msgs
  lanelet2_core lanelet2_io lanelet2_projection lanelet2_routing lanelet2_traffic_rules)
rclcpp_components_register_nodes(${PROJECT_NAME}
  "frenet_planner_ros2::PlannerNode" "frenet_planner_ros2::ControllerNode")

add_executable(planner_node src/main.cpp)
target_link_libraries(planner_node ${PROJECT_NAME})
add_executable(controller_node src/controller_main.cpp)
target_link_libraries(controller_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS planner_node controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ ${COMMON_DIR}/include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/bag_replay_eval.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_planner_node test/test_planner_node.cpp TIMEOUT 120)
  target_link_libraries(test_planner_node ${PROJECT_NAME})
  ament_target_dependencies(test_planner_node ament_index_cpp)
endif()

ament_package()
