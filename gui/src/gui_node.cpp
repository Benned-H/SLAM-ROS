// Author: Benned Hedegaard
// Last revised 5/18/2020

#include "gui/gui.h"

using namespace std;

int main(int argc, char* argv[])
{
	GUI gui(0.3); // Input: Robot diameter (m)
	
	ros::init(argc, argv, "gui");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	ros::Subscriber pose_sub = node_handle.subscribe("simulator/pose", 1,
		&GUI::handlePose, &gui);
		
	gui.marker_pub = node_handle.advertise<visualization_msgs::Marker>
		("visualization_marker", 1 , true);
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
