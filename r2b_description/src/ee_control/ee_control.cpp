#include "ee_control.h"

EEControl::EEControl(ros::NodeHandle &_nh){
	g_connect_box_rqst.data = 0;
	g_attach_srv_rqst_sent = false;	// initially, nothing is attached
	g_detach_srv_rqst_sent = true;
	connection_status_.can_connect_to = 0;
	attacher_.request.model_name_1 = "r2b";
	attacher_.request.link_name_1 = "ee_link";
	attacher_.request.link_name_2 = "base_link";

	nh_ = _nh;
	connection_status_pub_ = nh_.advertise<r2b_utility::EeConnectionStatus>("/r2b/connection_status", 10);
	connect_box_sub_	= nh_.subscribe("/r2b/connect_box", 10, &EEControl::connectBoxCallback, this);
	attacher_srv_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
	detacher_srv_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

	node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
	node_->Init();
	box1_misalign_sub_ = node_->Subscribe("/r2b/box_top/misalignment", misalignBox1Callback);
	box2_misalign_sub_ = node_->Subscribe("/r2b/box_mid/misalignment", misalignBox2Callback);
	box3_misalign_sub_ = node_->Subscribe("/r2b/box_down/misalignment", misalignBox3Callback);
}

void EEControl::connectBoxCallback(const std_msgs::Bool::ConstPtr &_msg){
	g_connection_mtx.lock();

	g_connect_box_rqst.data = _msg->data;

	g_connection_mtx.unlock();
}

void misalignBox1Callback(ConstPoseStampedPtr &_msg){
	g_box1_mtx.lock();

	double x = _msg->pose().position().x();
	double y = _msg->pose().position().y();
	double z = _msg->pose().position().z();
	g_dist_wrt1 = calculateEuclideanDistance(x,y,z);

	// ROS_INFO_STREAM("w.r.t. 1: " << g_dist_wrt1);

	g_box1_mtx.unlock();
}

void misalignBox2Callback(ConstPoseStampedPtr &_msg){
	g_box2_mtx.lock();

	double x = _msg->pose().position().x();
	double y = _msg->pose().position().y();
	double z = _msg->pose().position().z();
	g_dist_wrt2 = calculateEuclideanDistance(x,y,z);
	
	// ROS_INFO_STREAM("w.r.t. 2: " << g_dist_wrt2);

	g_box2_mtx.unlock();
}

void misalignBox3Callback(ConstPoseStampedPtr &_msg){
	g_box3_mtx.lock();

	double x = _msg->pose().position().x();
	double y = _msg->pose().position().y();
	double z = _msg->pose().position().z();
	g_dist_wrt3 = calculateEuclideanDistance(x,y,z);

	// ROS_INFO_STREAM("w.r.t. 3: " << g_dist_wrt3);

	g_box3_mtx.unlock();
}

float roundThreeDecimals(float _var){
	float temp;
	if (_var < 0) temp = -_var;
	else temp = _var;
	float value = (int)(temp * 1000 + 0.5);
	return  copysign((float)value / 1000, _var);
}

float calculateEuclideanDistance(double &_x, double &_y, double &_z){
	float distance = 0;
	distance = sqrt(pow(_x, 2) + pow(_y, 2) + pow(_z, 2));
	return roundThreeDecimals(distance);
}

int main(int _argc, char **_argv){
	// initate gazebo stuff
	gazebo::client::setup(_argc, _argv);

	// initiate ROS stuff
	ros::init(_argc, _argv, "EEControl");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	// Loop update rate (Hz)
	ros::Rate loop_rate(10);

	// initiate control stuff
	EEControl ee_control(nh);

	// main loop

	while(ros::ok()){
		g_connection_mtx.lock();
		g_box1_mtx.lock();
		g_box2_mtx.lock();
		g_box3_mtx.lock();

		if(g_dist_wrt1 <= g_dist_wrt2){
			if(g_dist_wrt1 <= g_dist_wrt3){
				if(g_dist_wrt1 <= CONNECTION_THRESHOLD){
					ee_control.connection_status_.can_connect_to = 1;
				}
				else{
					ee_control.connection_status_.can_connect_to = 0;
				}
			}
			else{
				if(g_dist_wrt3 <= CONNECTION_THRESHOLD){
					ee_control.connection_status_.can_connect_to = 3;
				}
				else{
					ee_control.connection_status_.can_connect_to = 0;
				}
			}
		}
		else{
			if(g_dist_wrt2 <= g_dist_wrt3){
				if(g_dist_wrt2 <= CONNECTION_THRESHOLD){
					ee_control.connection_status_.can_connect_to = 2;
				}
				else{
					ee_control.connection_status_.can_connect_to = 0;
				}
			}
			else{
				if(g_dist_wrt3 <= CONNECTION_THRESHOLD){
					ee_control.connection_status_.can_connect_to = 3;
				}
				else{
					ee_control.connection_status_.can_connect_to = 0;
				}
			}
		}

		if(g_connect_box_rqst.data){
			if(!g_attach_srv_rqst_sent && ee_control.connection_status_.can_connect_to != 0){
				
				switch(ee_control.connection_status_.can_connect_to){
					case 1: ee_control.attacher_.request.model_name_2 = "box_top";
							break;
					case 2: ee_control.attacher_.request.model_name_2 = "box_mid";
							break;
					case 3: ee_control.attacher_.request.model_name_2 = "box_down";
							break;
				}

				if(ee_control.attacher_srv_client_.call(ee_control.attacher_)){
					ROS_INFO_STREAM("Attach request successful!");
				}
				else{
					ROS_ERROR_STREAM("Attach request failed!");
				}

				g_attach_srv_rqst_sent = true;
				g_detach_srv_rqst_sent = false;
			}
		}
		else{
			if(!g_detach_srv_rqst_sent){

				if(ee_control.detacher_srv_client_.call(ee_control.attacher_)){
					ROS_INFO_STREAM("Detach request successful!");
				}
				else{
					ROS_ERROR_STREAM("Detach request failed!");
				}

				g_attach_srv_rqst_sent = false;
				g_detach_srv_rqst_sent = true;
			}
		}

		ee_control.connection_status_.connection_rqst_sent = g_connect_box_rqst.data;

		g_connection_mtx.unlock();
		g_box1_mtx.unlock();
		g_box2_mtx.unlock();
		g_box3_mtx.unlock();

		ee_control.connection_status_pub_.publish(ee_control.connection_status_);

		loop_rate.sleep();
	}

	gazebo::client::shutdown();
	return 0;
}