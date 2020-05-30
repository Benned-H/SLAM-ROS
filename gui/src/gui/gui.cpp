// Author: Benned Hedegaard
// Last revised 5/29/2020

#include "gui/gui.h"

using namespace std;

GUI::GUI(double diameter) // Constructor
{
	ROBOT_DIAMETER = diameter;
	hasPath = false;
	hasLookahead = false;
}

GUI::~GUI() {} // Deconstructor

void GUI::handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	_pose = msg->pose.pose;
	update();
	return;
}

void GUI::handlePath(const planner::Path::ConstPtr& msg)
{
	_path = *msg;
	hasPath = true;
	update();
	return;
}

void GUI::handleLookaheadPoint(const geometry_msgs::Point::ConstPtr& msg)
{
	_lookahead = *msg;
	hasLookahead = true;
	update();
	return;
}
// We should only need to draw the static obstacles once.
void GUI::handleObstacles(const simulator::Obstacles::ConstPtr& msg)
{
	_obstacles = *msg;
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
	if (hasPath) drawPath(_path, 0.847, 0.204, 0.922);
	if (hasLookahead) drawLookahead(_lookahead, 0.1, 0.0, 0.8, 1.0);
	if (_obstacles.data.size() > 0) drawObstacles(1.0, 0.1, 0.1);
	
	return;
}

void GUI::drawPose(const geometry_msgs::Pose pose, double r, double g, double b)
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
	c.color = color(r,g,b,1.0);
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

void GUI::drawLookahead(geometry_msgs::Point point, double radius, double r,
			double g, double b)
{
	visualization_msgs::Marker p;
	p.header.frame_id = "/gui";
	p.header.stamp = ros::Time::now();
	p.ns = "gui";
	p.id = 3;
	p.type = 2;
	p.action = visualization_msgs::Marker::ADD;
	p.scale.x = 0.03;
	p.scale.y = 0.03;
	p.scale.z = 0.03;
	p.color = color(r,g,b,0.75);
	p.lifetime = ros::Duration(); // Never auto-deletes.
	p.pose.position = point;
	
	marker_pub.publish(p);
	
	return;
}

void GUI::drawObstacles(double r, double g, double b)
{
	for (int i = 0; i < _obstacles.data.size(); i++)
	{
		visualization_msgs::Marker cylinder; // Cylinder
		geometry_msgs::Point p = _obstacles.data[i];
		
		cylinder.header.frame_id = "/gui";
		cylinder.header.stamp = ros::Time::now();
		cylinder.ns = "gui";
		cylinder.id = 100 + i; // We id the obstacles as 100 onwards.
		cylinder.type = 3;
		cylinder.action = visualization_msgs::Marker::ADD;
		cylinder.pose.position.x = p.x;
		cylinder.pose.position.y = p.y;
		cylinder.pose.position.z = 0.25; // Offset to make their bottoms at 0.
		cylinder.pose.orientation.x = 0.0;
		cylinder.pose.orientation.y = 0.0;
		cylinder.pose.orientation.z = 0.0;
		cylinder.pose.orientation.w = 1.0;
		cylinder.scale.x = 2.0*p.z; // The diameter in the x/y direction.
		cylinder.scale.y = 2.0*p.z;
		cylinder.scale.z = 0.5;
		cylinder.color = color(r,g,b,0.9);
		cylinder.lifetime = ros::Duration(); // Never auto-deletes.
	
		marker_pub.publish(cylinder);
	}
	
	return;
}
