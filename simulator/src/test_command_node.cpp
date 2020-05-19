// Author: Benned Hedegaard
// Last revised 5/18/2020

#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "test_command_node");
	ros::NodeHandle node_handle;
	
	ros::Publisher command_pub = node_handle.advertise<geometry_msgs::Twist>
		("motion_commands", 1, true);
	
	geometry_msgs::Twist msg;
	msg.linear.x = 0.5;
	msg.angular.z = M_PI/4.0;
	command_pub.publish(msg);
	
	ros::spin();	
	
	return 0;
}
