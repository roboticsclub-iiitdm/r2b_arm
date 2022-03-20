add_executable(example_task main.cpp 
							   ${COMMON_SIM_SRC_FILES}
							   ${COMMON_ARDUINO_TYPE_INTERFACE_SRC_FILES})

target_link_libraries(example_task ${catkin_LIBRARIES})
target_include_directories(example_task PRIVATE ${COMMON_SRC_FILE_DIR})