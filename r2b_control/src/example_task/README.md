# r2b_control/src/example_task

## About

Contains an example task implemented using the Arduino-like interface. The following steps describe the easiest way to create your own `main.cpp` containing your own logic:

1. Make a copy of this directory in the `./r2b_control/src` directory and provide a descriptive name (e.g. *another_task*).
2. Add a new line in `CMakeLists.txt` file located in the `./r2b_control/src` directory that tells CMake to add the newly created directory as a sub-directory for the CMake project. (e.g. `add_subdirectory(another_task)`).
3. Goto the `CMakeLists.txt` file in the newly created directory and replace executable name `example_task` with a descriptive name for your executable (e.g. `another_task_exe`).
4. Modify the contents of the `main.cpp` file in the newly created directory as per your convinience, but ensure that `setup()` and `loop()` functions are not left undefined.