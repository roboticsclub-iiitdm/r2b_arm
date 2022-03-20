#ifndef R2B_SIMCONTROLLER_H
#define R2B_SIMCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class R2bSimController{

private:

/// @brief Flag variable indicating whether box attach/detach request is sent or not
std_msgs::Bool box_attach_rqst_sent_;

/// @brief Joint angle message
std_msgs::Float64 joint_angle_;

/// @brief Joint 1 publisher
ros::Publisher jnt_1_pub_;

/// @brief Joint 2 publisher
ros::Publisher jnt_2_pub_;

/// @brief Box attach/detach request publisher
ros::Publisher box_attach_pub_;

public:

	/**
	 * @brief Construct a new R2bSimController object (default)
	 * 
	 */
	R2bSimController(){}

	/**
	 * @brief Construct a new R2bSimController object
	 * 
	 * @param nh Reference to ROS node handle
	 */
	R2bSimController(ros::NodeHandle &nh);

	/**
	 * @brief Set r2b robot's joint angles.
	 * 
	 * @param _jnt_1 Angle in degrees
	 * @param _jnt_2 Angle in degrees
	 */
	void setJointAngles(double _jnt_1, double _jnt_2);

	/**
	 * @brief Toggle the grasping of the cube.
	 * 
	 */
	void toggleGrasp();

};

#endif