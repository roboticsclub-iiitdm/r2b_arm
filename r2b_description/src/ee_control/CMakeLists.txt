add_executable(ee_control ee_control.cpp)
target_link_libraries(ee_control ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)