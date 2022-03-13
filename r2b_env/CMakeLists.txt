cmake_minimum_required(VERSION 3.0.2)
project(r2b_env)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
