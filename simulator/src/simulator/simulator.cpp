// Author: Benned Hedegaard
// Last revised 5/18/2020

#include "geometry_msgs/Quaternion.h"

// Include the header we're defining methods for.
#include "simulator/simulator.h"

using namespace std;

/*
	u(0) - v (linear velocity forwards)
	u(1) - w (angular velocity)
	
	x(0) - x position
	x(1) - y position
	x(2) - theta (heading)
*/

Simulator::Simulator()
{
	_u(0) = 0.0;
	_u(1) = 0.0;
	
	_x(0) = 0.0;
	_x(1) = 0.0;
	_x(2) = 0.0;
	
	return;
}

Simulator::~Simulator() {}	// Deconstructor

void Simulator::handleMotionCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
	_u(0) = msg->linear.x;
	_u(1) = msg->angular.z;
	
	return;
}

geometry_msgs::Quaternion yaw_to_quat(double yaw)
{
	geometry_msgs::Quaternion q;
	q.w = cos(yaw/2.0);
	q.x = 0.0;
	q.y = 0.0;
	q.z = sin(yaw/2.0);
	
	return q;
}

nav_msgs::Odometry Simulator::getOdometry(void)
{
	nav_msgs::Odometry msg;
	msg.header.stamp = ros::Time::now();
	msg.pose.pose.position.x = _x(0);
	msg.pose.pose.position.y = _x(1);
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation = yaw_to_quat(_x(2));
	msg.twist.twist.linear.x = _u(0);
	msg.twist.twist.angular.z = _u(1);
	
	return msg;
}

// From Probabilistic Robotics Version 1 pg. 127.
void Simulator::step(double dt)
{
	double v = _u(0);
	double w = _u(1);
	if (w == 0.0) // Avoid dividing by 0.
		w = 0.00001;
	
	double new_x = _x(0) - (v/w)*sin(_x(2)) + (v/w)*sin(_x(2)+w*dt);
	double new_y = _x(1) + (v/w)*cos(_x(2)) - (v/w)*cos(_x(2)+w*dt);
	double new_theta = _x(2) + w*dt;
	
	_x(0) = new_x;
	_x(1) = new_y;
	_x(2) = new_theta;
	
	return;
}

