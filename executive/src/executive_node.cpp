// Author: Benned Hedegaard
// Last revised 5/23/2020

#include "executive/executive.h"

using namespace std;

int main(int argc, char* argv[])
{
	Executive exec(0.05, 0.5); // Inputs are reached and replan cutoffs.
	
	ros::init(argc, argv, "executive_node");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.

	ros::Subscriber odom_sub = node_handle.subscribe("simulator/odom", 1,
		&Executive::handleOdom, &exec);
	ros::Subscriber waypoint_sub = node_handle.subscribe("executive/waypoint", 1,
		&Executive::handleWaypoint, &exec);
	
	// Set up any publishers inside the class instance.
	exec.query_pub = node_handle.advertise<planner::Query>
		("planner/query", 1, true);

	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
