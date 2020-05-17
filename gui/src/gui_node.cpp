// Author: Benned Hedegaard
// Last revised 5/16/2020

#include "gui/gui.h"

using namespace std;

int main(int argc, char* argv[])
{
	GUI gui(0.3); // Input: Robot diameter
	
	ros::init(argc, argv, "gui");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	/*ros::Subscriber datatype_sub = node_handle.subscribe("topic_name", 1,
		&ClassName::handleMessageType, &obj);*/
	ros::Publisher marker_pub = node_handle.advertise
		<visualization_msgs::Marker>("visualization_marker", 1 , true);
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	double frequency = 20.0; // Desired rate of the timer in hz
	ros::Rate timer(frequency);
	
	while (ros::ok()) // Keeps looping as long as the node is running.
	{
		gui.update();	
		timer.sleep();
	}

	return 0;
}
