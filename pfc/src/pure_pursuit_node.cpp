// Author: Benned Hedegaard
// Last revised 5/21/2020

#include "pfc/purepursuit.h"

using namespace std;

int main(int argc, char* argv[])
{
	// Inputs are lookahead distance, required heading difference to drive to
	// a goal point, and default forward/turning velocities.
	PurePursuit pfc(0.2, M_PI/4.0, 0.5, M_PI/2.0);
	
	ros::init(argc, argv, "pure_pursuit_node");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	ros::Subscriber path_sub = node_handle.subscribe("planner/path", 1,
		&PurePursuit::handlePath, &pfc);
	
	// Set up any publishers inside the class instance.
	pfc.command_pub = node_handle.advertise<geometry_msgs::Twist>
		("motion_commands", 1, true);
	pfc.goal_point_pub = node_handle.advertise<geometry_msgs::Point>
		("pfc/goal_point", 1, true);
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
