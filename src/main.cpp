#include "hexacopter.h"

using namespace hxcpt;

int main(int argc, char** argv){

	ros::init(argc, argv, "sssup_hexacpt");

   	ros::NodeHandle n;

   	ROS_INFO("Start hexacopter control node");

	hexacopter drone(&n, true); // Verbose mode ON 

	ros::spin();

	ROS_INFO("End hexacopter control node");

	// ros::spin();

	return 0;
}