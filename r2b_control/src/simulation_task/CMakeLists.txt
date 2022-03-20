add_executable(simulation_task main.cpp 
							   ${COMMON_SIM_SRC_FILES}
							   ${COMMON_ARDUINO_TYPE_INTERFACE_SRC_FILES})

target_link_libraries(simulation_task ${catkin_LIBRARIES})
target_include_directories(simulation_task PRIVATE ${COMMON_SRC_FILE_DIR})