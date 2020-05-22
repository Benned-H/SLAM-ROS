// Author: Benned Hedegaard
// Last revised 5/21/2020

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

class Simulator
{
	public: // These data members can be accessed by other classes.
		Simulator(); // Constructor
		virtual ~Simulator(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleMotionCommand(const geometry_msgs::Twist::ConstPtr& msg);
		
		void step(double dt);
		nav_msgs::Odometry getOdometry(void);
		
	protected: // These data members are inaccessible outside the class.
		// Also include static member variables. Start names with underscores.
		Eigen::Vector2d _u; // Current motion command.
		Eigen::Vector3d _x; // Current robot pose.
};

#endif /* SIMULATOR_H */
