cmake_minimum_required(VERSION 3.16)
project(fiss_planner_msgs)

set(MSG_FILES
  ObstacleState.msg Obstacle.msg ObstacleArray.msg
  TrajectoryPoint.msg Trajectory.msg PlannerStats.msg
  StopLineState.msg StopLineStateArray.msg)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
  add_message_files(DIRECTORY msg FILES ${MSG_FILES})
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
else()
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  list(TRANSFORM MSG_FILES PREPEND "msg/")
  rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES std_msgs)
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
