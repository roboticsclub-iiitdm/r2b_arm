cmake_minimum_required(VERSION 3.0.2)
project(r2b_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)
