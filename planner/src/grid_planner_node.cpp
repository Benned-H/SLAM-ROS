// Author: Benned Hedegaard
// Last revised 8/27/2020

#include "planner/gridplanner.h"
#include "mapper/occmapper.h"

// TODO - Commented things out to check OccMapper building. Refactor needed!

int main(int argc, char* argv[])
{
	geometry_msgs::Pose origin;
	origin.position.x = -5.0;
	origin.position.y = -5.0;
	origin.position.z = 0.0;
	origin.orientation.x = 0.0;
	origin.orientation.y = 0.0;
	origin.orientation.z = 0.0;
	origin.orientation.w = 1.0;
	
	//GridPlanner planner(0.2, origin); // Input: discretization (m) and cost map origin
	
	ros::init(argc, argv, "planner_node");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	/*ros::Subscriber query_sub = node_handle.subscribe("planner/query", 1,
		&GridPlanner::handleQuery, &planner);
	ros::Subscriber map_sub = node_handle.subscribe("mapper/map", 1,
		&GridPlanner::handleMap, &planner);*/
	
	// Set up any publishers inside the class instance.
	/*planner.path_pub = node_handle.advertise<planner::Path>
		("planner/path", 1, true);
	planner.open_list_size_pub = node_handle.advertise<std_msgs::UInt32>
		("planner/open_list_size", 1, true);
	planner.closed_list_size_pub = node_handle.advertise<std_msgs::UInt32>
		("planner/closed_list_size", 1, true);*/
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
