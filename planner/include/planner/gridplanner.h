// Author: Benned Hedegaard
// Last revised 5/21/2020

#ifndef GRID_PLANNER_H
#define GRID_PLANNER_H

#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "planner/Query.h"
#include "planner/Path.h"

using namespace std;

class GridPlanner
{
	public: // These data members can be accessed by other classes.
		GridPlanner(double discretization); // Constructor
		virtual ~GridPlanner(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleQuery(const planner::Query::ConstPtr& msg);
		
		ros::Publisher path_pub;
		
		double DISCRETIZATION;
		
	protected: // These data members are inaccessible outside the class.
		vector<geometry_msgs::Point> aStar(geometry_msgs::Point start, geometry_msgs::Point goal);
};

#endif /* GRID_PLANNER_H */
