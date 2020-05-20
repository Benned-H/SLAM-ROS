// Author: Benned Hedegaard
// Last revised 5/19/2020

#include "gui/gui.h"

using namespace std;

GUI::GUI(double diameter) // Constructor
{
	ROBOT_DIAMETER = diameter;
}

GUI::~GUI() {} // Deconstructor

void GUI::handlePose(const nav_msgs::Odometry::ConstPtr& msg)
{
	_pose = msg->pose.pose;
	update();
	return;
}

void GUI::handlePath(const planner::Path::ConstPtr& msg)
{
	_path = *msg;
	update();
	return;
}

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
	drawPose(_pose, 0.0, 1.0, 0.0);
	drawPath(_path, 0.847, 0.204, 0.922);
	
	return;
}

void GUI::drawPose(geometry_msgs::Pose pose, double r, double g, double b)
{
	visualization_msgs::Marker c; // cylinder for robot body
	c.header.frame_id = "/gui";
	c.header.stamp = ros::Time::now();
	c.ns = "gui";
	c.id = 0;
	c.type = 3;
	c.action = visualization_msgs::Marker::ADD;
	c.pose = pose;
	c.scale.x = ROBOT_DIAMETER;
	c.scale.y = ROBOT_DIAMETER;
	c.scale.z = 0.15;
	c.color = color(r,g,b,0.5);
	c.lifetime = ros::Duration(); // Never auto-deletes.
	
	visualization_msgs::Marker a; // arrow for robot heading
	a.header.frame_id = "/gui";
	a.header.stamp = ros::Time::now();
	a.ns = "gui";
	a.id = 1;
	a.type = 0;
	a.action = visualization_msgs::Marker::ADD;
	a.pose = pose;
	a.scale.x = 0.3;
	a.scale.y = 0.05;
	a.scale.z = 0.05;
	a.color = color(r,g,b,1.0);
	a.lifetime = ros::Duration(); // Never auto-deletes.
		
	marker_pub.publish(c);
	marker_pub.publish(a);
	
	return;
}

void GUI::drawPath(planner::Path path, double r, double g, double b)
{
	visualization_msgs::Marker p; // line strip of points
	p.header.frame_id = "/gui";
	p.header.stamp = ros::Time::now();
	p.ns = "gui";
	p.id = 2;
	p.type = 4;
	p.action = visualization_msgs::Marker::ADD;
	p.scale.x = 0.03;
	p.color = color(r,g,b,0.75);
	p.lifetime = ros::Duration(); // Never auto-deletes.
	p.points = path.points;
	
	marker_pub.publish(p);
	
	return;
}
