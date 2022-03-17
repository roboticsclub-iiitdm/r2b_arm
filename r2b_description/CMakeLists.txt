cmake_minimum_required(VERSION 3.0.2)
project(r2b_description)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	gazebo_ros
	r2b_utility
)

catkin_package(CATKIN_DEPENDS roscpp r2b_utility)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(src/ee_control)