#include "r2b_simcontroller.h"

R2bSimController::R2bSimController(ros::NodeHandle &nh){
	is_cube_grasped_ = false;
	jnt_1_pub_ = nh.advertise<std_msgs::Float64>("/r2b/jnt_1_controller/command", 1);
	jnt_2_pub_ = nh.advertise<std_msgs::Float64>("/r2b/jnt_2_controller/command", 1);
}

void R2bSimController::setJointAngles(double _jnt_1 = 0, double _jnt_2 = 0){
	joint_angle_.data = _jnt_1;
	jnt_1_pub_.publish(joint_angle_);
	
	joint_angle_.data = _jnt_2;
	jnt_2_pub_.publish(joint_angle_);
}