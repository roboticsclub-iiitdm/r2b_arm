# r2b_control

## About

Main package containing nodes, and launch files for execution of control algorithms, and simulations.

## Commands

1. To start the Gazebo simulation with the r2b robot.
    ```bash
    roslaunch r2b_control r2b_bringup.launch
    ```
2. To control each joint of the r2b robot using GUI
    ```bash
    rosrun r2b_control r2b_joint_control.py
    ```