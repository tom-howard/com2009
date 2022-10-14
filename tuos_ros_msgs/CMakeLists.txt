cmake_minimum_required(VERSION 3.0.2)
project(tuos_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  NumpyInt32Array.msg
  NumpyFloat32Array.msg
)

add_service_files(
  FILES
  SetBool.srv
  TimedMovement.srv
  Approach.srv
)

add_action_files(
  FILES
  CameraSweep.action
  Search.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  rospy
  message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
