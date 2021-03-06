#include "r2b_simcontroller.h"

R2bSimController::R2bSimController(ros::NodeHandle &nh){
	box_attach_rqst_sent_.data = false;

	jnt_1_pub_ = nh.advertise<std_msgs::Float64>("/r2b/jnt_1_controller/command", 1, true);
	jnt_2_pub_ = nh.advertise<std_msgs::Float64>("/r2b/jnt_2_controller/command", 1, true);
	box_attach_pub_ = nh.advertise<std_msgs::Bool>("/r2b/connect_box", 1, true);
}

void R2bSimController::setJointAngles(double _jnt_1 = 0, double _jnt_2 = 0){
	joint_angle_.data = (-90 + _jnt_1) * DEG2RAD;
	jnt_1_pub_.publish(joint_angle_);
	
	joint_angle_.data = _jnt_2 * DEG2RAD;
	jnt_2_pub_.publish(joint_angle_);
	ros::spinOnce();
}

void R2bSimController::toggleGrasp(){
	if(box_attach_rqst_sent_.data){
		box_attach_rqst_sent_.data = false;
	}
	else{
		box_attach_rqst_sent_.data = true;
	}

	box_attach_pub_.publish(box_attach_rqst_sent_);
	ros::spinOnce();
}