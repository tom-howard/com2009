cmake_minimum_required(VERSION 3.0.2)
project(com2009_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  SetBool.srv
  TimedMovement.srv
  Approach.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
