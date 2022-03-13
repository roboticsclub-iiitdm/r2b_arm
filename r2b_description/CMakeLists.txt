cmake_minimum_required(VERSION 3.0.2)
project(r2b_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})