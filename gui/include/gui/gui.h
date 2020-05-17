// Author: Benned Hedegaard
// Last revised 5/16/2020

#ifndef GUI_H
#define GUI_H

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"

class GUI
{
	public: // These data members can be accessed by other classes.
		GUI(double diameter); // Constructor; could have input variables.
		virtual ~GUI(); // Deconstructor
		
		std_msgs::ColorRGBA color(double r, double g, double b, double a);
		void update();
		
		//void handleMessageType(const package_name::DataType::ConstPtr& msg);
		
		ros::Publisher marker_pub;
		
		double ROBOT_DIAMETER;
		
	protected: // These data members are inaccessible outside the class.
		void drawPose(geometry_msgs::Pose pose);
		
		geometry_msgs::Pose _pose;
};

#endif /* GUI_H */
