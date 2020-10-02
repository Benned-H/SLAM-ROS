// Author: Benned Hedegaard
// Last revised 8/27/2020

#ifndef OCC_MAPPER_H
#define OCC_MAPPER_H

#include <set>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/LaserScan.h"

class OccMapper
{
	public: // These data members can be accessed by other classes.
		OccMapper(double res, int threshold, unsigned int width, unsigned int height,
	double obstacle_width, geometry_msgs::Pose origin, double p0,
	double p_free, double p_occ);
		virtual ~OccMapper(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
		void handleLaserscan(const sensor_msgs::LaserScan::ConstPtr& msg);
		void handleClick(const geometry_msgs::PointStamped::ConstPtr& msg);
		
		void publishMap();
		bool occupied(double x, double y);
		
		// Declare any ROS publishers.
		ros::Publisher map_pub;
		
		double RESOLUTION;
		double MIN_X;
		double MAX_X;
		double MIN_Y;
		double MAX_Y;
		
		double l0; // Prior log-odds of occupancy.
		double l_occ; // Log-odds of occupied cell given laser hit.
		double l_free; // Log-odds of free cell given laser passes through.
		
		nav_msgs::OccupancyGrid _map;
		
	protected: // These data members are inaccessible outside the class.
		int x_to_col(double x);
		int y_to_row(double y);
		int point_to_index(double x, double y);
		bool inMap(double x, double y);
		void update(geometry_msgs::Pose pose, sensor_msgs::LaserScan scan);
		
		geometry_msgs::Pose _pose;
		sensor_msgs::LaserScan _scan;
		int _occ_steps; // Depth of obstacles in steps of size RESOLUTION.
		int _threshold; // We consider cells with this value or greater occupied.
};

#endif /* OCC_MAPPER_H */
