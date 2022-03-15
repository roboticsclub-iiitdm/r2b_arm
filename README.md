# r2b_arm

## About

A metapackage containing ROS packages that aid in control, simulation, and visualization of the **r2b_arm** developed for robotics training during Step Sessions conducted by the Robotics Club, IIITDM Kancheepuram.

## List of ROS packages

| Package Name | Brief description |
| --- | --- |
| [r2b_control](./r2b_control) | Main package containing nodes, and launch files for execution of control algorithms, and simulations. | 
| [r2b_description](./r2b_description) | Contains resources describing the geometry, actuators, and sensors of the robot. |
| [r2b_env](./r2b_env) | Contains resources describing the Gazebo environment for simulation. |
| [gazebo_ros_link_attacher](./gazebo_ros_link_attacher) | Utility ROS package added as a submodule to aid in simulating grasp. |

## Setup

To setup the repository onto your local machine:
```bash
# navigate to the src directory of your catkin workspace
git clone --recursive https://github.com/roboticsclub-iiitdm/r2b_arm.git
```
