#include "ee_control.h"

EEControl::EEControl(ros::NodeHandle &_nh){
	this->nh_ = _nh;
	this->connection_status_pub_ = this->nh_.advertise<r2b_utility::EeConnectionStatus>("/r2b/connection_status", 10);
	this->connect_box_sub_	= this->nh_.subscribe("/r2b/connect_box", 10, &EEControl::connectBoxCallback, this);

	this->node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
	this->node_->Init();
	this->box1_misalign_sub_ = this->node_->Subscribe("/r2b/box_top/misalignment", misalignBox1Callback);
	this->box2_misalign_sub_ = this->node_->Subscribe("/r2b/box_mid/misalignment", misalignBox2Callback);
	this->box3_misalign_sub_ = this->node_->Subscribe("/r2b/box_down/misalignment", misalignBox3Callback);
}

void EEControl::connectBoxCallback(const std_msgs::Bool::ConstPtr &_msg){
	ROS_INFO_STREAM("Stuff" << _msg->data);
}

void misalignBox1Callback(ConstPoseStampedPtr &_msg){
	ROS_INFO_STREAM("Misalign 1");
}

void misalignBox2Callback(ConstPoseStampedPtr &_msg){
	ROS_INFO_STREAM("Misalign 2"); 
}

void misalignBox3Callback(ConstPoseStampedPtr &_msg){
	ROS_INFO_STREAM("Misalign 3"); 
}

int main(int _argc, char **_argv){
	// initate gazebo stuff
	gazebo::client::setup(_argc, _argv);

	// initiate ROS stuff
	ros::init(_argc, _argv, "EEControl");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// initiate control stuff
	EEControl ee_control(nh);

	ros::waitForShutdown();
	gazebo::client::shutdown();
	return 0;
}