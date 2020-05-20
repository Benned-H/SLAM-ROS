// Author: Benned Hedegaard
// Last revised 5/19/2020

#ifndef GUI_H
#define GUI_H

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "planner/Path.h"
#include "visualization_msgs/Marker.h"

class GUI
{
	public: // These data members can be accessed by other classes.
		GUI(double diameter); // Constructor; could have input variables.
		virtual ~GUI(); // Deconstructor
		
		void handlePose(const nav_msgs::Odometry::ConstPtr& msg);
		void handlePath(const planner::Path::ConstPtr& msg);
		
		std_msgs::ColorRGBA color(double r, double g, double b, double a);
		void update();
		
		ros::Publisher marker_pub;
		
		double ROBOT_DIAMETER;
		
	protected: // These data members are inaccessible outside the class.
		void drawPose(geometry_msgs::Pose pose, double r, double g, double b);
		void drawPath(planner::Path path, double r, double g, double b);
		
		geometry_msgs::Pose _pose;
		planner::Path _path;
};

#endif /* GUI_H */
