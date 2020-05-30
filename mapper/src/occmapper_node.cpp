// Author: Benned Hedegaard
// Last revised 5/29/2020

#include "mapper/occmapper.h"

using namespace std;

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

	/* Constructor. Inputs are grid resolution, width/height of grid in # cells,
	origin of the map, and probability of a cell being occupied given:
	prior, laser thinks it's free, laser thinks it's occupied. */
	OccMapper occ(0.1, 100, 100, 0.2, origin, 0.3, 0.05, 0.95);
	
	ros::init(argc, argv, "occmapper_node");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.

	// Set up any subscribers
	ros::Subscriber odom_sub = node_handle.subscribe("simulator/odom", 1,
		&OccMapper::handleOdom, &occ);
	ros::Subscriber laserscan_sub = node_handle.subscribe("scan", 3,
		&OccMapper::handleLaserscan, &occ); // Stores up to 3 laserscans.
	/*ros::Subscriber click_sub = node_handle.subscribe("clicked_point", 1,
		&OccMapper::handleClick, &occ);*/ // Helps debug grid locations.
	
	// Set up any publishers inside the class instance.
	occ.map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>
		("mapper/map", 1, true);
		
	occ.publishMap();

	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
			
	ros::spin();
	
	return 0;
}
