// Author: Benned Hedegaard
// Last revised 5/22/2020

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "planner/Path.h"

class PurePursuit
{
	public: // These data members can be accessed by other classes.
		PurePursuit(double lookahead, double turn_angle, double forward_v,
			double turning_w);
		virtual ~PurePursuit(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
		void handlePath(const planner::Path::ConstPtr& msg);
		
		// Declare any ROS publishers.
		ros::Publisher command_pub;
		ros::Publisher goal_point_pub;
		
		double LOOKAHEAD; // Lookahead distance in the pure pursuit algorithm.
		double TURNING_ANGLE; // Turn robot until goals are within this angle.
		double DEFAULT_V; // Default driving speed.
		double DEFAULT_W; // Default turning speed.
		
	protected: // These data members are inaccessible outside the class.
		double quat_to_yaw(geometry_msgs::Quaternion q);
		geometry_msgs::Point goalPoint(geometry_msgs::Pose pose, planner::Path path);
		void purePursuit();
		
		nav_msgs::Odometry _odom;
		planner::Path _path;
		
		bool hasOdom;
		bool hasPath;
};

#endif /* PURE_PURSUIT_H */
