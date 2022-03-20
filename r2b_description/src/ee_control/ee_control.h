#ifndef EE_CONTROL_H
#define EE_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <r2b_utility/EeConnectionStatus.h>
#include <gazebo_ros_link_attacher/Attach.h>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <mutex>

#define CONNECTION_THRESHOLD 0.005

/// @brief Mutexes for thread safe sharing
std::mutex g_box1_mtx;
std::mutex g_box2_mtx;
std::mutex g_box3_mtx;
std::mutex g_connection_mtx;

/**
 * @brief Custom function to round decimals to three places
 * 
 * @param _var Value to be rounded
 * @return float 
 */
float roundThreeDecimals(float _var);

/**
 * @brief Calculates the Euclidean distance of a point from origin
 * 
 * @param _x
 * @param _y 
 * @param _z 
 * @return float Euclidean distance (rounded to 3rd decimal place) 
 */
float calculateEuclideanDistance(double &_x, double &_y, double &_z);

class EEControl{
	private:
	
	// ROS related >>>

	/// @brief ROS node-handle to get access to rosparams
	ros::NodeHandle nh_;

	/// @brief Subscriber to /r2b/connect_box topic; connects or disconnects box
	ros::Subscriber connect_box_sub_;

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

	/// @brief Publisher to /r2b/connection_status; utility for current status of EE frame w.r.t. each of the boxes
	ros::Publisher connection_status_pub_;

	/// @brief ROS service client that aids in attaching cubes
	ros::ServiceClient attacher_srv_client_;

	/// @brief ROS service client that aids in attaching cubes
	ros::ServiceClient detacher_srv_client_;

	/// @brief Handles service requests
	gazebo_ros_link_attacher::Attach attacher_;

	/// @brief Message containing status of connection of box with EE frame
	r2b_utility::EeConnectionStatus connection_status_;

	/**
	 * @brief Construct a new EEControl object
	 * 
	 * @param _nh Reference to the ROS node handle
	 */
	EEControl(ros::NodeHandle &_nh);

};
// Gazebo related >>>

/// @brief Euclidean distance EE frame w.r.t. box 1
float g_dist_wrt1;

/// @brief Euclidean distance EE frame w.r.t. box 2
float g_dist_wrt2;

/// @brief Euclidean distance EE frame w.r.t. box 3
float g_dist_wrt3;

/// @brief Request sent by user to connect/disconnect box
std_msgs::Bool g_connect_box_rqst;

/// @brief Attach request sent to service server
bool g_attach_srv_rqst_sent;

/// @brief Detach request sent to service server
bool g_detach_srv_rqst_sent;

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