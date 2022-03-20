#include "arduinotypeinterface.h"

bool task_done = false;

void setup(void){
	ROS_INFO_STREAM("Example task: This task will go towards the top box, and then go to the place where it needs to be placed");
}

void loop(void){
	if(!task_done){
		ROS_INFO_STREAM("Starting the task...");
		delay(1000);

		ROS_INFO_STREAM("Reached pose 1");
		r2b.setJointAngles(56.2, -47.56);
		delay(2000);

		ROS_INFO_STREAM("Reached pose 2");
		r2b.setJointAngles(90, -50);
		delay(2000);

		ROS_INFO_STREAM("Reached pose 3");
		r2b.setJointAngles(41.87, -89.38);
		delay(2000);

		ROS_INFO_STREAM("Task done");
		task_done = true;
	}
}