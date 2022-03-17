# r2b_description/src/em_ee

## About

Contains source files that implements the electromagnetic end-effector strategy for the r2b robot.

## List of topics

| Topic | Topic type | Sub/Pub | Use-case | 
| --- | --- | --- | --- |
| `/r2b/connect_box` | `ros::std_msgs::Bool` | Sub | Used to connect or disconnect the box with the end-effector.|
| `/r2b/connection_status` | `r2b_utility::EeConnectionStatus` | Pub | For broadcasting the status of the distance of boxes from the EE frame and which box can be connected.|
| `/r2b/box_top/misalignment` | `gazebo::msgs::PoseStamped` | Sub | Obtains the pose of EE frame w.r.t. box 1. | 
| `/r2b/box_mid/misalignment` | `gazebo::msgs::PoseStamped` | Sub | Obtains the pose of EE frame w.r.t. box 2. | 
| `/r2b/box_down/misalignment` | `gazebo::msgs::PoseStamped` | Sub | Obtains the pose of EE frame w.r.t. box 3. | 