// Author: Benned Hedegaard
// Last revised 5/22/2020

// Include the header we're defining methods for.
#include "executive/executive.h"

using namespace std;

Executive::Executive(double reached, double replan) // Constructor
{
	REACHED = reached;
	REPLAN = replan;
	hasOdom = false;
}

Executive::~Executive() {} // Deconstructor

double euclidean(double x1, double y1, double x2, double y2)
{
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

void Executive::handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	_odom = *msg;
	hasOdom = true;
	
	if (_waypoints.size() == 0) // Exit if there are no more waypoints.
		return;
		
	// Check if we've reached the current waypoint. If so, remove.
	double xr = _odom.pose.pose.position.x;
	double yr = _odom.pose.pose.position.y;
	geometry_msgs::Point curr_waypoint = _waypoints.front();
	
	if (euclidean(xr, yr, curr_waypoint.x, curr_waypoint.y) < REACHED)
	{
		_waypoints.erase(_waypoints.begin());
		if (_waypoints.size() == 0) // Exit if there are no more waypoints.
			return;
		curr_waypoint = _waypoints.front();
	}
	
	// If we're close to the current waypoint, don't create a new plan.
	if (euclidean(xr, yr, curr_waypoint.x, curr_waypoint.y) < REPLAN)
		return;
	
	sendQuery();
		
	return;
}

void Executive::handleWaypoint(const geometry_msgs::Point::ConstPtr& msg)
{
	_waypoints.push_back(*msg);
	
	if (_waypoints.size() == 1 && hasOdom) // New waypoint is our first.
		sendQuery();
	
	return;
}

// This function will always send a new query based on the first waypoint.
void Executive::sendQuery()
{	
	planner::Query q;
	q.start = _odom.pose.pose.position;
	q.goal = _waypoints.front();
	
	query_pub.publish(q);
	return;
}
