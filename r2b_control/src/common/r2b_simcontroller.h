#ifndef R2B_SIMCONTROLLER_H
#define R2B_SIMCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class R2bSimController{

private:

/// @brief Flag variable indicating whether cube is grasped or not
bool is_cube_grasped_;

/// @brief Joint angle message
std_msgs::Float64 joint_angle_;

/// @brief Joint 1 publisher
ros::Publisher jnt_1_pub_;

/// @brief Joint 2 publisher
ros::Publisher jnt_2_pub_;

public:
	/**
	 * @brief Construct a new R2bSimController object
	 * 
	 * @param nh Reference to ROS node handle
	 */
	R2bSimController(ros::NodeHandle &nh);

	/**
	 * @brief Destroy the R2bSimController object
	 */
	~R2bSimController();

	/**
	 * @brief Set r2b robot's joint angles.
	 * 
	 * @param _jnt_1 Angle in degrees
	 * @param _jnt_2 Angle in degrees
	 */
	void setJointAngles(double _jnt_1 = 0, double _jnt_2 = 0);

	/**
	 * @brief Toggle the grasping of the cube.
	 * 
	 */
	void toggleGrasp();

};

#endif