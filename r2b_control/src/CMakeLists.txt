set(COMMON_SRC_FILE_DIR ${PROJECT_SOURCE_DIR}/src/common)
set(COMMON_SIM_SRC_FILES  ${COMMON_SRC_FILE_DIR}/r2b_simcontroller.cpp)
set(COMMON_ARDUINO_TYPE_INTERFACE_SRC_FILES 
		${COMMON_SRC_FILE_DIR}/arduinotypeinterface.cpp
		${COMMON_SRC_FILE_DIR}/arduinotypeinterface.h)

add_subdirectory(example_task)
add_subdirectory(simulation_task)