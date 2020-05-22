// Author: Benned Hedegaard
// Last revised 5/21/2020

#include "planner/gridplanner.h"

using namespace std;

int main(int argc, char* argv[])
{
	GridPlanner planner(0.15); // Input: discretization (m)
	
	ros::init(argc, argv, "planner_node");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	ros::Subscriber query_sub = node_handle.subscribe("planner/query", 1,
		&GridPlanner::handleQuery, &planner);
	
	// Set up any publishers inside the class instance.
	planner.path_pub = node_handle.advertise<planner::Path>
		("planner/path", 1, true);
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
