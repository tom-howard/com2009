cmake_minimum_required(VERSION 3.0.2)
project(tuos_ros_simulations)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
