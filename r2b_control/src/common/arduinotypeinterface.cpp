#include "arduinotypeinterface.h"

R2bSimController r2b;

void delay(long millis){
	ros::Duration(millis/1000).sleep();
}

int main(int _argc, char **_argv){

	// initiate ROS stuff
	ros::init(_argc, _argv, "arduino_type_interface", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Loop update rate (Hz)
	ros::Rate loop_rate(10);

	r2b = R2bSimController(nh);

	setup();

	while(ros::ok()){
		loop();
		loop_rate.sleep();
	}

}