// Author: Benned Hedegaard
// Last revised 5/16/2020

#include "gui/gui.h"

using namespace std;

GUI::GUI(double diameter) // Constructor
{
	ROBOT_DIAMETER = diameter;
}

GUI::~GUI() {} // Deconstructor

/*
void GUI::handleMessageType(const package_name::DataType::ConstPtr& msg)
{
	_data = *msg;
	_data2 = msg->position.x; // Might want to access subparts of the message.
	
	// We could call other functions or publish new information if we wanted.
	return;
}*/

std_msgs::ColorRGBA GUI::color(double r, double g, double b, double a)
{
	std_msgs::ColorRGBA c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
	return c;
}

void GUI::update()
{
	drawPose(_pose);
	
	return;
}

void GUI::drawPose(geometry_msgs::Pose pose)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/gui";
	marker.header.stamp = ros::Time::now();
	marker.ns = "gui";
	marker.id = 0;
	marker.type = 3;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.scale.x = ROBOT_DIAMETER;
	marker.scale.y = ROBOT_DIAMETER;
	marker.scale.z = 0.15;
	marker.color = color(0.0,1.0,0.0,1.0);
	marker.lifetime = ros::Duration(); // Never auto-deletes.
		
	marker_pub.publish(marker);
}
