cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  control_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_bringup
#  CATKIN_DEPENDS actionlib amcl control_msgs geometry_msgs move_base_msgs nav_msgs robot_state_publisher roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_Cmd_Vel_to_Arduino src/cmd_vel_mux.cpp)
set_target_properties(${PROJECT_NAME}_Cmd_Vel_to_Arduino PROPERTIES OUTPUT_NAME Cmd_Vel_to_Arduino PREFIX "")

add_dependencies(${PROJECT_NAME}_Cmd_Vel_to_Arduino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_Cmd_Vel_to_Arduino
   ${catkin_LIBRARIES}
 )

add_executable(${PROJECT_NAME}_Mecanum_TF src/odometry.cpp)
set_target_properties(${PROJECT_NAME}_Mecanum_TF PROPERTIES OUTPUT_NAME Mecanum_TF PREFIX "")

add_dependencies(${PROJECT_NAME}_Mecanum_TF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_Mecanum_TF
 ${catkin_LIBRARIES}
)

#catkin_install_python(PROGRAMS src/send_goal.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#catkin_install_python(PROGRAMS src/send_goal_action.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

catkin_install_python(PROGRAMS src/receive_goal_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
