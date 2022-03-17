#ifndef EE_CONTROL_H
#define EE_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <r2b_utility/EeConnectionStatus.h>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

class EEControl{
	private:
	
	// ROS related >>>

	/// @brief ROS node-handle to get access to rosparams
	ros::NodeHandle nh_;

	/// @brief Subscriber to /r2b/connect_box topic; connects or disconnects box
	ros::Subscriber connect_box_sub_;

	/// @brief Publisher to /r2b/connection_status; utility for current status of EE frame w.r.t. each of the boxes
	ros::Publisher connection_status_pub_;

	/**
	 * @brief Callback function informing whether to connect or disconnect the 
	 * box.
	 * @param _msg Boolean: True or False
	 */
	void connectBoxCallback(const std_msgs::Bool::ConstPtr &_msg);

	// Gazebo related >>>

	/// @brief Gazebo transport node for communication
	gazebo::transport::NodePtr node_;

	/// @brief Gazebo subscriber to box 1 misaligned pose topic
	gazebo::transport::SubscriberPtr box1_misalign_sub_;

	/// @brief Gazebo subscriber to box 2 misaligned pose topic
	gazebo::transport::SubscriberPtr box2_misalign_sub_;
	
	/// @brief Gazebo subscriber to box 3 misaligned pose topic
	gazebo::transport::SubscriberPtr box3_misalign_sub_;
	
	public:

	/**
	 * @brief Construct a new EEControl object
	 * 
	 * @param _nh Reference to the ROS node handle
	 */
	EEControl(ros::NodeHandle &_nh);

};
// Gazebo related >>>

/**
 * @brief Callback function informing about the pose of EE frame w.r.t. box1.
 * 
 * @param _msg Pose
 */
void misalignBox1Callback(ConstPoseStampedPtr &_msg);

/**
 * @brief Callback function informing about the pose of EE frame w.r.t. box2.
 * 
 * @param _msg Pose
 */
void misalignBox2Callback(ConstPoseStampedPtr &_msg);

/**
 * @brief Callback function informing about the pose of EE frame w.r.t. box3.
 * 
 * @param _msg Pose
 */
void misalignBox3Callback(ConstPoseStampedPtr &_msg);

#endif