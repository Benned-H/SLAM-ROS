// Author: Benned Hedegaard
// Last revised 5/21/2020

#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "planner/Query.h"

class Executive
{
	public: // These data members can be accessed by other classes.
		Executive(double reached, double replan); // Constructor
		virtual ~Executive(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
		void handleWaypoint(const geometry_msgs::Point::ConstPtr& msg);
		
		void sendQuery();
		
		// Declare any ROS publishers.
		ros::Publisher query_pub;
		
		double REACHED; // Within this distance counts as reaching a waypoint.
		double REPLAN; // Don't replan within this distance.
		
	protected: // These data members are inaccessible outside the class.
		nav_msgs::Odometry _odom;
		std::vector<geometry_msgs::Point> _waypoints;
};

#endif /* EXECUTIVE_H */
