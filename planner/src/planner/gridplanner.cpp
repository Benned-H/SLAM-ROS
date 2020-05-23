// Author: Benned Hedegaard
// Last revised 5/23/2020

#include <math.h>

#include "planner/gridplanner.h"

using namespace std;

GridPlanner::GridPlanner(double discretization) // Constructor
{
	DISCRETIZATION = discretization;
}

GridPlanner::~GridPlanner() {} // Deconstructor

void GridPlanner::handleQuery(const planner::Query::ConstPtr& msg)
{
	planner::Path p;
	p.points = aStar(msg->start, msg->goal);
	path_pub.publish(p);
	return;
}

class Node
{
public:
	Node(int _x, int _y);
	int x; // Discrete locations in the grid; (0,0) is the origin.
	int y;
	double g;
	double f;
	struct Node* prev;
};
#define NODE Node*

Node::Node(int _x, int _y)
{
	x = _x;
	y = _y;
	g = 0.0;
	f = 0.0;
	prev = NULL;
	return;
}

double euclidean(double x1, double y1, double x2, double y2)
{
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

// Computes real-world distance between discretized integer points.
double discrete_euclidean(int x1, int y1, int x2, int y2, int d)
{
	return d * sqrt( (double)((x1-x2)*(x1-x2)) + (double)((y1-y2)*(y1-y2)) );
}


// Sorts nodes in ascending order by their f value.
bool compNodes(NODE n1, NODE n2)
{
	return (n1->f < n2->f);
}

// Runs A* on a grid with the stored discretization.
vector<geometry_msgs::Point> GridPlanner::aStar(geometry_msgs::Point start, geometry_msgs::Point goal)
{
	// First create the open and closed lists.
	vector<NODE> open_list;
	vector<NODE> closed_list;

	// Start from the closest on-grid point.
	Node s0(round(start.x/DISCRETIZATION), round(start.y/DISCRETIZATION));
	s0.g = 0.0;
	s0.f = euclidean(s0.x, s0.y, goal.x, goal.y);
	open_list.push_back(&s0);
	
	std_msgs::UInt32 open_list_size; // For debugging purposes.
	std_msgs::UInt32 closed_list_size;
	// Main loop as long as the open list contains nodes.
	while (open_list.size() > 0)
	{
		open_list_size.data = open_list.size();
		closed_list_size.data = closed_list.size();
		open_list_size_pub.publish(open_list_size);
		closed_list_size_pub.publish(closed_list_size);
	
		// Pop best node, put into closed list.
		NODE curr = open_list.front();
		open_list.erase(open_list.begin());
		closed_list.push_back(curr);

		// If we've almost reached the goal, extract and return the path.
		if (euclidean((curr->x)*DISCRETIZATION, (curr->y)*DISCRETIZATION,
			goal.x, goal.y) <= DISCRETIZATION)
		{
			vector<geometry_msgs::Point> path;
			if ((curr->x) != goal.x && (curr->y) != goal.y)
				path.push_back(goal);
			geometry_msgs::Point p;
			p.x = (curr->x)*DISCRETIZATION;
			p.y = (curr->y)*DISCRETIZATION;
			path.insert(path.begin(), p);

			while (curr->prev != NULL)
			{
				curr = curr->prev;
				geometry_msgs::Point p;
				p.x = (curr->x)*DISCRETIZATION;
				p.y = (curr->y)*DISCRETIZATION;
				path.insert(path.begin(), p);
			}

			return path;
		}

		// Consider the valid actions from the current node.
		vector<NODE> neighbors;
		neighbors.push_back(new Node(curr->x+1, curr->y));
		neighbors.push_back(new Node(curr->x+1, curr->y+1));
		neighbors.push_back(new Node(curr->x, curr->y+1));
		neighbors.push_back(new Node(curr->x-1, curr->y+1));
		neighbors.push_back(new Node(curr->x-1, curr->y));
		neighbors.push_back(new Node(curr->x-1, curr->y-1));
		neighbors.push_back(new Node(curr->x, curr->y-1));
		neighbors.push_back(new Node(curr->x+1, curr->y-1));

		for (int i = 0; i < neighbors.size(); i++)
		{
			NODE new_node = neighbors[i];

			bool closed = false;
			for (int c = 0; c < closed_list.size(); c++)
			{
				if ((closed_list[c])->x == new_node->x && (closed_list[c])->y == new_node->y)
				{
					closed = true;
					break;
				}
			}

			if (closed) // Don't expand already-closed nodes.
				continue;

			// Set new node's g and f values and its previous node.
			new_node->g = curr->g + discrete_euclidean(new_node->x, new_node->y, curr->x, curr->y, DISCRETIZATION);
			new_node->f = new_node->g + euclidean(DISCRETIZATION*new_node->x, DISCRETIZATION*new_node->y, goal.x, goal.y);
			new_node->prev = curr;

			// Push to the open list.
			open_list.push_back(new_node);
		}
		sort(open_list.begin(), open_list.end(), compNodes);
	}

	// If the main while loop failed, we've lost. Return empty list.
	vector<geometry_msgs::Point> output;
	return output;
}

