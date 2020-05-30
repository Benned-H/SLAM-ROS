// Author: Benned Hedegaard
// Last revised 5/29/2020

#include <stdlib.h>

#include "ros/ros.h"
#include "simulator/Obstacles.h"
#include "geometry_msgs/Point.h"

using namespace std;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "obstacle_publisher_node");
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	ros::Publisher obstacles_pub = node_handle.advertise<simulator::Obstacles>
		("simulator/obstacles", 1, true);
	
	srand(time(NULL)); // Initialize random seed using current time.
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	simulator::Obstacles msg;
	
	int NUMBER_OF_OBSTACLES = 50;
	int successes = 0;
	while (successes < NUMBER_OF_OBSTACLES)
	{
		geometry_msgs::Point p;
		p.x = (rand() % 1001)*0.01 - 5.0; // Between -5.0 and 5.0
		p.y = (rand() % 1001)*0.01 - 5.0;
		int rng = rand() % 451; // RNG 0 to 450
		p.z = 0.05 + rng*0.001; // Will be 0.05 to 0.5. Radius of the obstacle.
		
		// Too close to the origin.
		if (sqrt(p.x*p.x + p.y*p.y) - p.z < 0.5)
		{
			continue;
		}
		else
		{
			bool skip = false;
			for (int i = 0; i < msg.data.size(); i++) // Check unique.
			{
				if (msg.data[i].x == p.x && msg.data[i].y == p.y)
				{
					skip = true;
					break;
				}
			}
			if (skip) continue;
			successes++;
			msg.data.push_back(p);
		}
	}
	
	obstacles_pub.publish(msg);
	
	ros::spin();
	
	return 0;
}
