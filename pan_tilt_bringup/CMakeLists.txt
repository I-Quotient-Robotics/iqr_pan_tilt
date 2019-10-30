cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_bringup)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
)

catkin_package(
#	CATKIN_DEPENDS message_runtime 
#  INCLUDE_DIRS include
#  LIBRARIES pan_tilt_bringup
#  CATKIN_DEPENDS dynamic_reconfigure message_generation roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)
